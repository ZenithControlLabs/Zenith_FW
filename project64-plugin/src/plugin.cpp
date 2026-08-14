#include "project64_controller.h"
#include "usb_device.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <commctrl.h>

namespace {
ZenithUsbDevice g_device;
HINSTANCE g_instance;
HWND g_mainWindow = nullptr;
std::atomic_bool g_connectionErrorShown{false};

constexpr int kPhysicalCount = 15;
const wchar_t *kPhysicalNames[kPhysicalCount] = {
    L"A", L"B", L"C Up", L"C Down", L"C Left", L"C Right", L"Start",
    L"L", L"R", L"Z / ZL", L"D Down", L"D Left", L"D Right", L"D Up", L"ZR"};
const wchar_t *kLogicalNames[] = {
    L"Unbound", L"A", L"B", L"C Up", L"C Down", L"C Left", L"C Right",
    L"Start", L"L", L"R", L"Z", L"D Down", L"D Left", L"D Right", L"D Up"};
const std::uint8_t kLogicalValues[] = {
    0xFF, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};

enum { ID_REFRESH = 100, ID_APPLY = 101, ID_STATUS = 102, ID_COMBO_BASE = 200 };
std::array<HWND, kPhysicalCount> g_combos{};
std::array<std::uint8_t, 32> g_mapping{};

void updateStatus(HWND window, const wchar_t *text) {
    SetWindowTextW(GetDlgItem(window, ID_STATUS), text);
}

void showConnectionError(HWND parent) {
    if (g_connectionErrorShown.exchange(true)) return;

    const DWORD error = g_device.lastError();
    const wchar_t *message =
        (error == ERROR_ACCESS_DENIED || error == ERROR_SHARING_VIOLATION)
        ? L"The Zenith controller could not be opened because another application is using its configuration interface.\n\n"
          L"Close the web configurator (or any other application using WebUSB), then reload the plugin or restart emulation."
        : L"The Zenith controller could not be opened. Make sure it is connected in Switch Pro mode, then reload the plugin or restart emulation.\n\n"
          L"If the web configurator is open, close it first because WebUSB access may be exclusive.";
    MessageBoxW(parent, message, L"Zenith Raw Input - Controller unavailable",
                MB_OK | MB_ICONERROR);
}

bool connectController(HWND parent, bool notifyOnFailure) {
    if (g_device.connect()) {
        g_connectionErrorShown.store(false);
        return true;
    }
    if (notifyOnFailure) showConnectionError(parent);
    return false;
}

void drawButton(const DRAWITEMSTRUCT &item) {
    const bool pressed = (item.itemState & ODS_SELECTED) != 0;
    const bool disabled = (item.itemState & ODS_DISABLED) != 0;
    const COLORREF background = disabled ? RGB(92, 92, 92)
                                         : pressed ? RGB(62, 62, 62) : RGB(42, 42, 42);
    HBRUSH brush = CreateSolidBrush(background);
    FillRect(item.hDC, &item.rcItem, brush);
    DeleteObject(brush);
    FrameRect(item.hDC, &item.rcItem, GetSysColorBrush(COLOR_3DSHADOW));

    wchar_t label[64]{};
    GetWindowTextW(item.hwndItem, label, static_cast<int>(std::size(label)));
    SetBkMode(item.hDC, TRANSPARENT);
    SetTextColor(item.hDC, disabled ? RGB(190, 190, 190) : RGB(255, 255, 255));
    HFONT font = reinterpret_cast<HFONT>(SendMessageW(item.hwndItem, WM_GETFONT, 0, 0));
    HGDIOBJ previousFont = font ? SelectObject(item.hDC, font) : nullptr;
    RECT textRect = item.rcItem;
    DrawTextW(item.hDC, label, -1, &textRect,
              DT_CENTER | DT_VCENTER | DT_SINGLELINE);
    if (previousFont) SelectObject(item.hDC, previousFont);
    if ((item.itemState & ODS_FOCUS) != 0) {
        RECT focusRect = item.rcItem;
        InflateRect(&focusRect, -3, -3);
        DrawFocusRect(item.hDC, &focusRect);
    }
}

void populateMapping(HWND window) {
    if (!g_device.readN64Mapping(g_mapping)) {
        updateStatus(window, L"Controller not found in Switch Pro mode");
        showConnectionError(window);
        return;
    }
    for (int source = 0; source < kPhysicalCount; ++source) {
        std::uint8_t value = g_mapping[source] == 0
            ? static_cast<std::uint8_t>(source + 1) : g_mapping[source];
        int selected = 0;
        for (int option = 0; option < static_cast<int>(std::size(kLogicalValues)); ++option)
            if (kLogicalValues[option] == value) selected = option;
        SendMessageW(g_combos[source], CB_SETCURSEL, selected, 0);
    }
    g_connectionErrorShown.store(false);
    updateStatus(window, L"Connected · mappings loaded from controller");
}

void applyMapping(HWND window) {
    for (int source = 0; source < kPhysicalCount; ++source) {
        int selected = static_cast<int>(SendMessageW(g_combos[source], CB_GETCURSEL, 0, 0));
        if (selected >= 0) g_mapping[source] = kLogicalValues[selected];
    }
    if (g_device.writeN64Mapping(g_mapping)) {
        g_connectionErrorShown.store(false);
        updateStatus(window, L"Saved to controller");
    } else {
        updateStatus(window, L"Save failed · reconnect in Switch Pro mode");
        showConnectionError(window);
    }
}

LRESULT CALLBACK configWindowProc(HWND window, UINT message, WPARAM wParam, LPARAM lParam) {
    (void)lParam;
    switch (message) {
    case WM_CREATE: {
        HFONT font = static_cast<HFONT>(GetStockObject(DEFAULT_GUI_FONT));
        HWND intro = CreateWindowW(L"STATIC", L"Each physical button maps to exactly one N64 button.",
            WS_CHILD | WS_VISIBLE, 18, 14, 430, 20, window, nullptr, g_instance, nullptr);
        SendMessageW(intro, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        for (int i = 0; i < kPhysicalCount; ++i) {
            int column = i / 8;
            int row = i % 8;
            int x = 18 + column * 245;
            int y = 47 + row * 36;
            HWND label = CreateWindowW(L"STATIC", kPhysicalNames[i], WS_CHILD | WS_VISIBLE,
                x, y + 5, 80, 22, window, nullptr, g_instance, nullptr);
            SendMessageW(label, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
            g_combos[i] = CreateWindowW(WC_COMBOBOXW, L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST,
                x + 84, y, 145, 280, window,
                reinterpret_cast<HMENU>(static_cast<INT_PTR>(ID_COMBO_BASE + i)), g_instance, nullptr);
            SendMessageW(g_combos[i], WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
            for (const wchar_t *name : kLogicalNames)
                SendMessageW(g_combos[i], CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(name));
        }
        HWND status = CreateWindowW(L"STATIC", L"Looking for controller…", WS_CHILD | WS_VISIBLE,
            18, 348, 470, 22, window, reinterpret_cast<HMENU>(ID_STATUS), g_instance, nullptr);
        SendMessageW(status, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        HWND refresh = CreateWindowW(L"BUTTON", L"Reload", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
            254, 382, 100, 30, window, reinterpret_cast<HMENU>(ID_REFRESH), g_instance, nullptr);
        HWND apply = CreateWindowW(L"BUTTON", L"Apply && Save", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
            364, 382, 124, 30, window, reinterpret_cast<HMENU>(ID_APPLY), g_instance, nullptr);
        SendMessageW(refresh, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        SendMessageW(apply, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
        populateMapping(window);
        return 0;
    }
    case WM_COMMAND:
        if (LOWORD(wParam) == ID_REFRESH) populateMapping(window);
        if (LOWORD(wParam) == ID_APPLY) applyMapping(window);
        return 0;
    case WM_DRAWITEM: {
        const auto *item = reinterpret_cast<const DRAWITEMSTRUCT *>(lParam);
        if (item && (item->CtlID == ID_REFRESH || item->CtlID == ID_APPLY)) {
            drawButton(*item);
            return TRUE;
        }
        return DefWindowProcW(window, message, wParam, lParam);
    }
    case WM_CLOSE:
        DestroyWindow(window);
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProcW(window, message, wParam, lParam);
    }
}

void showConfig(HWND parent) {
    const wchar_t *className = L"ZenithRawInputConfig";
    WNDCLASSEXW wc{sizeof(wc)};
    wc.lpfnWndProc = configWindowProc;
    wc.hInstance = g_instance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
    wc.lpszClassName = className;
    RegisterClassExW(&wc);

    HWND window = CreateWindowExW(WS_EX_DLGMODALFRAME, className,
        L"Zenith Raw Input · Button Mapping", WS_CAPTION | WS_SYSMENU,
        CW_USEDEFAULT, CW_USEDEFAULT, 525, 460, parent, nullptr, g_instance, nullptr);
    if (!window) return;
    EnableWindow(parent, FALSE);
    ShowWindow(window, SW_SHOW);
    MSG message;
    while (IsWindow(window) && GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    EnableWindow(parent, TRUE);
    SetForegroundWindow(parent);
}
}

BOOL APIENTRY DllMain(HINSTANCE module, DWORD reason, LPVOID) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_instance = module;
        DisableThreadLibraryCalls(module);
    }
    return TRUE;
}

EXPORT void CALL GetDllInfo(PLUGIN_INFO *info) {
    if (!info) return;
    std::memset(info, 0, sizeof(*info));
    info->Version = 0x0101;
    info->Type = PLUGIN_TYPE_CONTROLLER;
    std::snprintf(info->Name, sizeof(info->Name), "Zenith Raw Input 1.0");
}

EXPORT void CALL InitiateControllers(CONTROL_INFO info) {
    g_mainWindow = info.hMainWindow;
    g_connectionErrorShown.store(false);
    for (int i = 0; i < 4; ++i) {
        info.Controls[i].Present = (i == 0) ? TRUE : FALSE;
        info.Controls[i].RawData = FALSE;
        info.Controls[i].Plugin = PLUGIN_NONE;
    }
    connectController(g_mainWindow, true);
}

EXPORT void CALL GetKeys(int control, BUTTONS *keys) {
    if (!keys) return;
    keys->Value = 0;
    if (control != 0) return;
    std::array<std::uint8_t, 4> state{};
    if (!g_device.readN64State(state)) {
        showConnectionError(g_mainWindow);
        return;
    }
    // Both the firmware's N64 report and Project64 consume X followed by Y.
    // Project64's legacy bitfield header labels these packed bytes in reverse.
    auto *bytes = reinterpret_cast<std::uint8_t *>(&keys->Value);
    bytes[0] = state[0];
    bytes[1] = state[1];
    bytes[2] = state[2];
    bytes[3] = state[3];
}

EXPORT void CALL DllConfig(HWND parent) { showConfig(parent); }
EXPORT void CALL DllAbout(HWND parent) {
    MessageBoxW(parent,
        L"Reads the controller's calibrated, corrected N64 signal directly over USB.\n\n"
        L"The controller must be in Switch Pro mode. Button mappings are stored on the controller.",
        L"Zenith Raw Input", MB_OK | MB_ICONINFORMATION);
}
EXPORT void CALL DllTest(HWND parent) {
    if (connectController(parent, false)) {
        MessageBoxW(parent, L"Controller connected.", L"Zenith Raw Input",
                    MB_OK | MB_ICONINFORMATION);
    } else {
        g_connectionErrorShown.store(false);
        showConnectionError(parent);
    }
}
EXPORT void CALL CloseDLL() { g_device.disconnect(); }
EXPORT void CALL RomOpen() { connectController(g_mainWindow, true); }
EXPORT void CALL RomClosed() {}
EXPORT void CALL ControllerCommand(int, BYTE *) {}
EXPORT void CALL ReadController(int, BYTE *) {}
EXPORT void CALL WM_KeyDown(WPARAM, LPARAM) {}
EXPORT void CALL WM_KeyUp(WPARAM, LPARAM) {}
