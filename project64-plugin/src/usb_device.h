#ifndef ZENITH_USB_DEVICE_H
#define ZENITH_USB_DEVICE_H

#include <array>
#include <cstdint>
#include <mutex>

#include <windows.h>
#include <winusb.h>

class ZenithUsbDevice {
public:
    ~ZenithUsbDevice();
    bool connect();
    void disconnect();
    bool connected() const;
    DWORD lastError() const;
    bool readN64State(std::array<std::uint8_t, 4> &state);
    bool readN64Mapping(std::array<std::uint8_t, 32> &mapping);
    bool writeN64Mapping(const std::array<std::uint8_t, 32> &mapping);

private:
    bool connectLocked();
    bool transactLocked(const std::uint8_t *request, ULONG requestSize,
                        std::uint8_t *response, ULONG responseSize);
    bool writeLocked(const std::uint8_t *request, ULONG requestSize);
    void disconnectLocked();

    mutable std::mutex mutex_;
    HANDLE file_ = INVALID_HANDLE_VALUE;
    WINUSB_INTERFACE_HANDLE usb_ = nullptr;
    ULONGLONG nextReconnectMs_ = 0;
    DWORD lastError_ = ERROR_SUCCESS;
};

#endif
