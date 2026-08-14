/*jshint esversion: 8 */
// @ts-check

import { sleep_ms } from "./utils.js";
import { placeCalibStatus } from "./zenith_calib.js";
import { updateInputDisplayRaw } from "./zenith_input.js";
import { placeGateLimiter, placeLpfCutoff, placeMagThresh, placeNotches } from "./zenith_notch.js";
import { RemapMode, placeRemapping, setRemapMode } from "./zenith_remap.js";

const filters = {filters: [{vendorId: 0x057E, productId: 0x2009}]};
const LIVE_INPUT_INTERVAL_MS = 8;

export const WebUSBCmdMap = {
    FW_GET: 0xA1,
    CALIBRATION_START: 0x01,
    CALIBRATION_ADVANCE: 0x02,
    CALIBRATION_UNDO: 0x03,
    CALIBRATION_STATUS_GET: 0xA2,
    NOTCH_SET: 0x04,
    NOTCHES_GET: 0xA4,
    REMAP_SET: 0x05,
    REMAP_GET: 0xA5,
    MAG_THRESH_SET: 0x06,
    MAG_THRESH_GET: 0xA6,
    GATE_LIMITER_SET: 0x07,
    GATE_LIMITER_GET: 0xA7,
    LPF_CUTOFF_SET: 0x08,
    LPF_CUTOFF_GET: 0xA8,
    COMMS_MODE_SET: 0x09,
    COMMS_MODE_GET: 0xA9,
    RAW_N64_GET: 0xAA,
    UPDATE_FW: 0xF1,
    COMMIT_SETTINGS: 0xF2,
    RESET_SETTINGS: 0xF3
};

export let usbDevice;
let listenGeneration = 0;
let rawTimer;
let rawWritePending = false;
let writeQueue = Promise.resolve();

const minimumResponseLength = {
    [WebUSBCmdMap.FW_GET]: 3,
    [WebUSBCmdMap.CALIBRATION_STATUS_GET]: 3,
    [WebUSBCmdMap.NOTCHES_GET]: 49,
    [WebUSBCmdMap.REMAP_GET]: 34,
    [WebUSBCmdMap.MAG_THRESH_GET]: 8,
    [WebUSBCmdMap.GATE_LIMITER_GET]: 2,
    [WebUSBCmdMap.LPF_CUTOFF_GET]: 8,
    [WebUSBCmdMap.COMMS_MODE_GET]: 2,
    [WebUSBCmdMap.RAW_N64_GET]: 16,
};

const disconnected = /** @type {HTMLDivElement} */ (document.getElementById("disconnect-div"));
const connected = /** @type {HTMLDivElement} */ (document.getElementById("connect-div"));
const saveIndicator = /** @type {HTMLSpanElement} */ (document.getElementById("save-indicator-span"));
const statusPill = document.getElementById("connection-status");

function showConnected(value) {
    disconnected.style.display = value ? "none" : "grid";
    connected.style.display = value ? "grid" : "none";
    statusPill.textContent = value ? "Connected" : "Disconnected";
    statusPill.classList.toggle("connected", value);
}

export async function connect() {
    try {
        // @ts-ignore
        const permitted = await navigator.usb.getDevices();
        usbDevice = permitted.find(device => device.vendorId === 0x057E && device.productId === 0x2009);
        if (!usbDevice) {
            // @ts-ignore
            usbDevice = await navigator.usb.requestDevice(filters);
        }
        await initWebUSBDevice();
        clearSaveIndicator();
        showConnected(true);
    } catch (error) {
        console.error(error);
        window.alert(`Could not connect to ${productName}. Make sure it is in Switch Pro mode, then try again.`);
    }
}

async function initWebUSBDevice() {
    if (!usbDevice.opened) await usbDevice.open();
    if (!usbDevice.configuration) await usbDevice.selectConfiguration(1);
    await usbDevice.claimInterface(1);

    const generation = ++listenGeneration;
    void listen(generation);
    await loadAllSettings();
    await loadVersion();
    clearInterval(rawTimer);
    rawWritePending = false;
    rawTimer = setInterval(() => {
        if (!usbDevice?.opened || rawWritePending) return;
        rawWritePending = true;
        void writeUSBCmd(WebUSBCmdMap.RAW_N64_GET)
            .catch(error => console.warn("Live-input request failed", error))
            .finally(() => { rawWritePending = false; });
    }, LIVE_INPUT_INTERVAL_MS);
}

export const writeUSBData = async (data) => {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    const queuedWrite = async () => {
        if (!usbDevice?.opened) return;
        const result = await usbDevice.transferOut(2, bytes);
        if (result.status !== "ok")
            throw new Error(`WebUSB write failed: ${result.status}`);
    };
    writeQueue = writeQueue.catch(() => {}).then(queuedWrite);
    return writeQueue;
};

export const writeUSBCmd = async (cmd) => writeUSBData(new Uint8Array([cmd]));

// @ts-ignore
navigator.usb.addEventListener("disconnect", event => {
    if (event.device !== usbDevice) return;
    usbDevice = null;
    ++listenGeneration;
    clearInterval(rawTimer);
    rawWritePending = false;
    showConnected(false);
});

async function listen(generation) {
    while (usbDevice?.opened && generation === listenGeneration) {
        try {
            const result = await usbDevice.transferIn(2, 64);
            if (result.status === "stall") {
                await usbDevice.clearHalt("in", 2);
                continue;
            }
            if (result.status !== "ok" || !result.data || result.data.byteLength === 0)
                continue;

            const command = result.data.getUint8(0);
            const minimumLength = minimumResponseLength[command] ?? 1;
            if (result.data.byteLength < minimumLength) {
                console.warn(`Ignoring short WebUSB response 0x${command.toString(16)} (${result.data.byteLength}/${minimumLength} bytes)`);
                continue;
            }

            switch (command) {
            case WebUSBCmdMap.CALIBRATION_STATUS_GET: placeCalibStatus(result.data); break;
            case WebUSBCmdMap.NOTCHES_GET: placeNotches(result.data); break;
            case WebUSBCmdMap.REMAP_GET: placeRemapping(result.data); break;
            case WebUSBCmdMap.MAG_THRESH_GET: placeMagThresh(result.data); break;
            case WebUSBCmdMap.GATE_LIMITER_GET: placeGateLimiter(result.data); break;
            case WebUSBCmdMap.LPF_CUTOFF_GET: placeLpfCutoff(result.data); break;
            case WebUSBCmdMap.COMMS_MODE_GET: placeCommsMode(result.data.getUint8(1)); break;
            case WebUSBCmdMap.RAW_N64_GET: updateInputDisplayRaw(result.data); break;
            case WebUSBCmdMap.FW_GET: placeVersion(result.data); break;
            }
        } catch (error) {
            if (generation !== listenGeneration || !usbDevice?.opened) break;
            console.warn("WebUSB read failed; retrying", error);
            await sleep_ms(100);
        }
    }
}

export function setSaveIndicator() { saveIndicator.style.display = "inline-flex"; }
function clearSaveIndicator() { saveIndicator.style.display = "none"; }

export async function saveSettings() {
    await writeUSBCmd(WebUSBCmdMap.COMMIT_SETTINGS);
    clearSaveIndicator();
}

export async function updateFw() { await writeUSBCmd(WebUSBCmdMap.UPDATE_FW); }

export async function resetSettings() {
    await writeUSBCmd(WebUSBCmdMap.RESET_SETTINGS);
    await sleep_ms(150);
    await loadAllSettings();
    setSaveIndicator();
}

export async function setOperatingMode(mode) {
    if (!Number.isInteger(mode) || mode < 0 || mode > 2) return;
    await writeUSBData(new Uint8Array([WebUSBCmdMap.COMMS_MODE_SET, mode]));
    setSaveIndicator();
    const notice = document.getElementById("mode-notice");
    notice.textContent = mode === 2
        ? "Save, then reconnect. At plug-in, hold A for Switch Pro or B for XInput; either shortcut saves that mode."
        : "The selected wired mode takes effect after reconnecting. Hold B while plugging in to select and save XInput directly.";
}

function placeCommsMode(mode) {
    const select = /** @type {HTMLSelectElement} */ (document.getElementById("operating-mode"));
    select.value = String(mode);
}

async function loadAllSettings() {
    const commands = [
        WebUSBCmdMap.CALIBRATION_STATUS_GET,
        WebUSBCmdMap.NOTCHES_GET,
        WebUSBCmdMap.MAG_THRESH_GET,
        WebUSBCmdMap.GATE_LIMITER_GET,
        WebUSBCmdMap.LPF_CUTOFF_GET,
        WebUSBCmdMap.COMMS_MODE_GET,
    ];
    for (const command of commands) {
        await writeUSBCmd(command);
        await sleep_ms(35);
    }
    await setRemapMode(RemapMode.Switch);
}

async function loadVersion() {
    await writeUSBCmd(WebUSBCmdMap.FW_GET);
}

function placeVersion(data) {
    const versionElem = document.getElementById("version-text");
    const major = data.getUint8(2);
    const minor = (data.getUint8(1) >> 4) & 0xF;
    const patch = data.getUint8(1) & 0xF;
    let commit = "";
    const commitLength = Math.min(60, data.byteLength - 3);
    for (let i = 0; i < commitLength && data.getUint8(3 + i) !== 0; ++i)
        commit += String.fromCharCode(data.getUint8(3 + i));
    versionElem.textContent = `Firmware ${major}.${minor}.${patch} · ${commit || "local build"}`;
}
