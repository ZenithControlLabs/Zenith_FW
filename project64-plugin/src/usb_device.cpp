#include "usb_device.h"

#include <setupapi.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace {
constexpr UCHAR kPipeOut = 0x02;
constexpr UCHAR kPipeIn = 0x82;
constexpr std::uint8_t kRawN64Get = 0xAA;
constexpr std::uint8_t kRemapSet = 0x05;
constexpr std::uint8_t kRemapGet = 0xA5;
constexpr std::uint8_t kCommit = 0xF2;
constexpr std::uint8_t kN64Map = 0;

// DeviceInterfaceGUIDs exposed by the firmware's Microsoft OS 2.0 descriptor.
const GUID kZenithInterfaceGuid =
    {0x8b3e9d2e, 0x7eec, 0x4994, {0xaa, 0xe7, 0x0c, 0x40, 0xde, 0x84, 0xd3, 0x6d}};
}

ZenithUsbDevice::~ZenithUsbDevice() { disconnect(); }

bool ZenithUsbDevice::connected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return usb_ != nullptr;
}

DWORD ZenithUsbDevice::lastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lastError_;
}

bool ZenithUsbDevice::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    return connectLocked();
}

bool ZenithUsbDevice::connectLocked() {
    if (usb_) return true;
    if (GetTickCount64() < nextReconnectMs_) return false;
    nextReconnectMs_ = GetTickCount64() + 1000;

    HDEVINFO info = SetupDiGetClassDevsW(&kZenithInterfaceGuid, nullptr, nullptr,
                                         DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (info == INVALID_HANDLE_VALUE) {
        lastError_ = GetLastError();
        return false;
    }

    SP_DEVICE_INTERFACE_DATA interfaceData{};
    interfaceData.cbSize = sizeof(interfaceData);
    if (!SetupDiEnumDeviceInterfaces(info, nullptr, &kZenithInterfaceGuid, 0,
                                     &interfaceData)) {
        lastError_ = GetLastError();
        SetupDiDestroyDeviceInfoList(info);
        return false;
    }

    DWORD required = 0;
    SetupDiGetDeviceInterfaceDetailW(info, &interfaceData, nullptr, 0,
                                     &required, nullptr);
    std::vector<std::uint8_t> storage(required);
    auto *detail = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA_W *>(storage.data());
    detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_W);
    if (!SetupDiGetDeviceInterfaceDetailW(info, &interfaceData, detail, required,
                                          nullptr, nullptr)) {
        lastError_ = GetLastError();
        SetupDiDestroyDeviceInfoList(info);
        return false;
    }

    file_ = CreateFileW(detail->DevicePath, GENERIC_READ | GENERIC_WRITE,
                        0, nullptr, OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
    SetupDiDestroyDeviceInfoList(info);
    if (file_ == INVALID_HANDLE_VALUE) {
        lastError_ = GetLastError();
        return false;
    }
    if (!WinUsb_Initialize(file_, &usb_)) {
        lastError_ = GetLastError();
        CloseHandle(file_);
        file_ = INVALID_HANDLE_VALUE;
        return false;
    }

    ULONG timeout = 50;
    WinUsb_SetPipePolicy(usb_, kPipeIn, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeout), &timeout);
    WinUsb_SetPipePolicy(usb_, kPipeOut, PIPE_TRANSFER_TIMEOUT,
                         sizeof(timeout), &timeout);
    lastError_ = ERROR_SUCCESS;
    return true;
}

void ZenithUsbDevice::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    disconnectLocked();
}

void ZenithUsbDevice::disconnectLocked() {
    if (usb_) WinUsb_Free(usb_);
    if (file_ != INVALID_HANDLE_VALUE) CloseHandle(file_);
    usb_ = nullptr;
    file_ = INVALID_HANDLE_VALUE;
}

bool ZenithUsbDevice::writeLocked(const std::uint8_t *request, ULONG size) {
    if (!connectLocked()) return false;
    ULONG written = 0;
    const BOOL success = WinUsb_WritePipe(usb_, kPipeOut,
                                          const_cast<PUCHAR>(request), size,
                                          &written, nullptr);
    if (!success || written != size) {
        lastError_ = success ? ERROR_BAD_LENGTH : GetLastError();
        disconnectLocked();
        return false;
    }
    return true;
}

bool ZenithUsbDevice::transactLocked(const std::uint8_t *request, ULONG requestSize,
                                     std::uint8_t *response, ULONG responseSize) {
    if (!writeLocked(request, requestSize)) return false;
    std::array<std::uint8_t, 64> packet{};
    ULONG read = 0;
    const BOOL success = WinUsb_ReadPipe(usb_, kPipeIn, packet.data(),
                                         static_cast<ULONG>(packet.size()),
                                         &read, nullptr);
    if (!success || read < responseSize) {
        lastError_ = success ? ERROR_BAD_LENGTH : GetLastError();
        disconnectLocked();
        return false;
    }
    std::copy_n(packet.data(), responseSize, response);
    return true;
}

bool ZenithUsbDevice::readN64State(std::array<std::uint8_t, 4> &state) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<std::uint8_t, 5> response{};
    if (!transactLocked(&kRawN64Get, 1, response.data(),
                        static_cast<ULONG>(response.size()))) return false;
    if (response[0] != kRawN64Get) {
        lastError_ = ERROR_INVALID_DATA;
        disconnectLocked();
        return false;
    }
    std::copy_n(response.begin() + 1, state.size(), state.begin());
    return true;
}

bool ZenithUsbDevice::readN64Mapping(std::array<std::uint8_t, 32> &mapping) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint8_t request[2] = {kRemapGet, kN64Map};
    std::array<std::uint8_t, 34> response{};
    if (!transactLocked(request, sizeof(request), response.data(),
                        static_cast<ULONG>(response.size()))) return false;
    if (response[0] != kRemapGet || response[1] != kN64Map) {
        lastError_ = ERROR_INVALID_DATA;
        disconnectLocked();
        return false;
    }
    std::copy_n(response.begin() + 2, mapping.size(), mapping.begin());
    return true;
}

bool ZenithUsbDevice::writeN64Mapping(const std::array<std::uint8_t, 32> &mapping) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::uint8_t source = 0; source < mapping.size(); ++source) {
        const std::uint8_t request[4] = {kRemapSet, kN64Map, source, mapping[source]};
        if (!writeLocked(request, sizeof(request))) return false;
        Sleep(2);
    }
    return writeLocked(&kCommit, 1);
}
