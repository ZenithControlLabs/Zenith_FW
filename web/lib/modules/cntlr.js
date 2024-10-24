/*jshint esversion: 6 */
// @ts-check

import { sleep_ms } from "./utils.js";
import { calStep } from "./zenith_calib.js";
import { updateInputDisplay } from "./zenith_input.js";
import { placeGateLimiter, placeMagThresh, placeNotches } from "./zenith_notch.js";
import { CommsMode, _commsMode, placeRemapping, setCommsMode } from "./zenith_remap.js";

const filters = {filters: [{
    vendorId: devVID,
    productId: devPID,
}]};


export const WebUSBCmdMap = {
    FW_GET: 0xA1,
    CALIBRATION_START: 0x01,
    CALIBRATION_ADVANCE: 0x02,
    CALIBRATION_UNDO: 0x03,
    CALIBRATION_STEP_GET: 0xA2,
    NOTCH_SET: 0x04,
    NOTCHES_GET: 0xA4,
    REMAP_SET: 0x05,
    REMAP_GET: 0xA5,
    MAG_THRESH_SET: 0x06,
    MAG_THRESH_GET: 0xA6,
    GATE_LIMITER_SET: 0x07,
    GATE_LIMITER_GET: 0xA7,
    UPDATE_FW: 0xF1,
    COMMIT_SETTINGS: 0xF2,
    RESET_SETTINGS: 0xF3
}

export let usbDevice;
export let hidDevice;

let disconnectDiv = /** @type {HTMLDivElement} */ (document.getElementById("disconnect-div"));
let connectDiv = /** @type {HTMLDivElement} */ (document.getElementById("connect-div"));
let saveIndicatorSpan = /** @type {HTMLSpanElement} */ (document.getElementById("save-indicator-span"));

function enableMenus(en) {
    disconnectDiv.style.display = en ? "none" : "flex";
    connectDiv.style.display = en ? "flex" : "none";
}

export async function connect() {
    // @ts-ignore
    let usbDevices = await navigator.usb.getDevices(filters);
    // @ts-ignore
    let hidDevices = await navigator.hid.getDevices(filters);

    if (usbDevices[0] && hidDevices[0]) {
        console.log("Already got device.");
        usbDevice = usbDevices[0];
        hidDevice = hidDevices[0];
    } else {
        console.log("Need device permission or not found.");
        // @ts-ignore
        usbDevice = await navigator.usb.requestDevice(filters);
        // @ts-ignore
        [hidDevice] = await navigator.hid.requestDevice(filters);
    }    

    if ((usbDevice == null) || (hidDevice == null)) {
        window.alert(`Please connect a valid ${productName} device.`);
        return;
    }

    try {
        await initWebUSBDevice();
        await initWebHIDDevice();
    } catch (e) {
        console.error(e);
        return;
    }
    
    clearSaveIndicator();
    enableMenus(true);
}

///////////////////
// WebHID logic //
/////////////////

async function initWebHIDDevice () {
    try {
        if (!hidDevice.opened) {
            await hidDevice.open();
        }
        hidDevice.addEventListener("inputreport", handleInputReport);
    } catch (error) {
        window.alert(`Could not initialize WebHID for ${productName} device`);
        throw error;
    }
        
}

function handleInputReport(event) {
    const {data, device, reportId} = event;
    
    if ((device.productId !== devPID) || device.vendorId !== devVID) return;
    
    if (reportId == 0x4) {
        updateInputDisplay(data);
    }
}

///////////////////
// WebUSB logic //
/////////////////

async function initWebUSBDevice () {
    try {
        await usbDevice.open();
        await usbDevice.selectConfiguration(1);
        await usbDevice.claimInterface(1);

        setInterval(() => {

            try {
                listen();
            }
            catch (err) {
                console.log(err);
            }
        }, 50);
        
    await loadAllSettings();
    await loadVersion();
    }
    catch (error) {
        window.alert(`Could not initialize WebUSB for ${productName} device`);
        throw error;
    }
}

export const writeUSBCmd = async (cmd) => { await usbDevice.transferOut(2, new Uint8Array([cmd])); }


// Set connect and disconnect listeners
// @ts-ignore
navigator.usb.addEventListener("connect", (event) => {
    console.log("Device plugged.");

});

// @ts-ignore
navigator.usb.addEventListener("disconnect", (event) => {
    console.log("Device unplugged.");
    if (event.device == usbDevice) {
        usbDevice = null;
        enableMenus(false);
    }
});

const listen = async () => {
    if (usbDevice != null) {
        const result = await usbDevice.transferIn(2, 64);

        switch (result.data.getUint8(0)) {
            case WebUSBCmdMap.NOTCHES_GET: {
                placeNotches(result.data);
                break;
            }
            case WebUSBCmdMap.REMAP_GET: {
                placeRemapping(result.data);
                break;
            }
            case WebUSBCmdMap.MAG_THRESH_GET: {
                placeMagThresh(result.data);
                break;
            }
            case WebUSBCmdMap.GATE_LIMITER_GET: {
                placeGateLimiter(result.data);
                break;
            }
            case WebUSBCmdMap.FW_GET: {
                placeVersion(result.data);
                break;
            }
        }
    }
}

export function setSaveIndicator() {
    saveIndicatorSpan.style.display = "block";
}

function clearSaveIndicator() {
    saveIndicatorSpan.style.display = "none";
}

export async function saveSettings() {
    await writeUSBCmd(WebUSBCmdMap.COMMIT_SETTINGS);
    clearSaveIndicator();
}

export async function resetSettings() {
    await writeUSBCmd(WebUSBCmdMap.RESET_SETTINGS);
    await sleep_ms(100);
    await loadAllSettings();
    setSaveIndicator();
}

async function loadAllSettings() {
    await writeUSBCmd(WebUSBCmdMap.NOTCHES_GET);
    await sleep_ms(50);
    await writeUSBCmd(WebUSBCmdMap.MAG_THRESH_GET);
    await sleep_ms(100);
    await writeUSBCmd(WebUSBCmdMap.GATE_LIMITER_GET);
    await sleep_ms(100);
    await setCommsMode(CommsMode.N64);
}

async function loadVersion() {
    await writeUSBCmd(WebUSBCmdMap.FW_GET);
    await sleep_ms(100);
}

function placeVersion(data) {
    let versionElem = /** @type {HTMLInputElement} */ (document.getElementById(`version-text`)); 
    let versionMaj = data.getUint8(2);
    let versionMin = (data.getUint8(1) >> 4) & 0xF;
    let versionPatch = data.getUint8(1) & 0xF;
    let i = 0;
    let commit = "";
    while (data.getUint8(3 + i) != 0 && i < 64) {
        commit += String.fromCharCode(data.getUint8(3+i));
        i++;
    }
    if (commit == "") { commit = "?" }
    versionElem.textContent = `Version: v${versionMaj}.${versionMin}.${versionPatch} (commit ${commit})`;
}