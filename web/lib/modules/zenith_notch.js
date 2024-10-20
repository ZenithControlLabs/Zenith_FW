/*jshint esversion: 6 */
// @ts-check

import { setSaveIndicator, usbDevice, WebUSBCmdMap } from "./cntlr.js";
import { IntToFloat32, swap32 } from "./utils.js";

function toDeadzoneAng(num) {
    return (num < 0) ? 0 : (num/180. * Math.PI);
}

function fromDeadzoneAng(num) {
    return num/(Math.PI) * 180.;
}

export async function updateNotchPoint(elemName) {
    let id_split = elemName.id.split("-");
    const ind = Number(id_split[1]);
    const type = id_split[2];
    let deadzonevalue;
    let yvalue;
    let xvalue;
    if (type === "x") {
        // @ts-ignore
        deadzonevalue = toDeadzoneAng(Number(document.getElementById(`notch-${ind}-deadzone`).value));
        // @ts-ignore
        yvalue = Number(document.getElementById(`notch-${ind}-y`).value);
        xvalue = Number(elemName.value);        
    } else if (type === "y") {
        // @ts-ignore
        deadzonevalue = toDeadzoneAng(Number(document.getElementById(`notch-${ind}-deadzone`).value));
        yvalue = Number(elemName.value);
        // @ts-ignore
        xvalue = Number(document.getElementById(`notch-${ind}-x`).value);
    } else if (type === "deadzone") {
        deadzonevalue = toDeadzoneAng(Number(elemName.value));
        // @ts-ignore
        yvalue = Number(document.getElementById(`notch-${ind}-y`).value);
        // @ts-ignore
        xvalue = Number(document.getElementById(`notch-${ind}-x`).value);

        // put it back, mainly just to verify that what we set to the controller is the same
        // as what the user sees in the textbox
        elemName.value = fromDeadzoneAng(deadzonevalue);
    } else {
        console.error("Unexpected element calling updateNotchPoint().");
        return;
    }
    // javascript buffer hackery to get the raw float representation in a byte array
    let buf = new ArrayBuffer(8);
    // First set the float in the array
    (new Float32Array(buf))[1] = deadzonevalue;
    // Now we can fill in the rest of the data
    buf = (new Uint8Array(buf));
    buf[0] = WebUSBCmdMap.NOTCH_SET;
    buf[1] = ind;
    buf[2] = xvalue;
    buf[3] = yvalue;
    console.log(buf);
    console.log(`id: ${ind}, x: ${xvalue}, y: ${yvalue}, ang: ${deadzonevalue}`);
    await usbDevice.transferOut(2, buf);
    setSaveIndicator();
}

export async function updateMagThresh(elemName) {
    let new_thresh = elemName.value;
    if (new_thresh < 0) {
        new_thresh = 0;
        elemName.value = 0;
    } else if (new_thresh > 100) {
        new_thresh = 100;
        elemName.value = 100;
    }
    let buf = new ArrayBuffer(8);
    (new Float32Array(buf))[1] = new_thresh / 100.;
    buf = (new Uint8Array(buf));
    buf[0] = WebUSBCmdMap.MAG_THRESH_SET;
    console.log(buf);
    await usbDevice.transferOut(2, buf);
}

export function placeNotches(data) {
    for (let i = 0; i < 8; i++) {
        let notchpoint_input_x = /** @type {HTMLInputElement} */ (document.getElementById(`notch-${i}-x`));
        let notchpoint_input_y = /** @type {HTMLInputElement} */ (document.getElementById(`notch-${i}-y`)); 
        let notchpoint_input_deadzone = /** @type {HTMLInputElement} */ (document.getElementById(`notch-${i}-deadzone`)); 
        notchpoint_input_x.value = data.getInt8(1 + 6*i);
        notchpoint_input_y.value = data.getInt8(2 + 6*i);
        notchpoint_input_deadzone.value = fromDeadzoneAng(IntToFloat32(swap32(data.getUint32(3 + 6*i)))).toFixed(3);
        console.log(`Setting ${i} to ${notchpoint_input_x.value}, ${notchpoint_input_y.value}, ${notchpoint_input_deadzone.value}`); 
    }
}

export function placeMagThresh(data) {
    let mag_thresh_input = /** @type {HTMLInputElement} */ (document.getElementById(`mag-thresh`)); 
    mag_thresh_input.value = (100. * IntToFloat32(swap32(data.getUint32(4)))).toFixed(2);
}