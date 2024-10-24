/*jshint esversion: 6 */
// @ts-check

import { WebUSBCmdMap, setSaveIndicator, usbDevice } from "./cntlr.js";

let remapDiv = /** @type {HTMLDivElement} */ (document.getElementById("bindings-fill-in"));

export const CommsMode = {
    N64: 0x00,
    Gamecube: 0x01
}

const N64Buttons = [
    "A",
    "B",
    "CU",
    "CD",
    "CL",
    "CR",
    "Start",
    "L",
    "R",
    "Z",
    "DD",
    "DL",
    "DR",
    "DU",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
]

export let _commsMode;

export async function setCommsMode(commsMode) {
    if (!(commsMode in Object.values(CommsMode))) {
        console.error("Tried to set illegal comms mode..");
    }
    await usbDevice.transferOut(2, new Uint8Array([WebUSBCmdMap.REMAP_GET, commsMode]));
    _commsMode = commsMode;
}


export async function setBinding() {
    const btn = buttonNames.indexOf(this.name);
    const bind = Number(this.value);
    if (btn > 32 || (bind != 0xFF && bind > 32)) {
        console.error("Internal button remap error");
        return;
    }
    await usbDevice.transferOut(2, new Uint8Array([WebUSBCmdMap.REMAP_SET, _commsMode, btn, bind]));
    setSaveIndicator();
}

function getRemappingOptionsElems(commsMode) {
    
    // doesnt actually exist in firmware yet
    let emptyOption = /** @type {HTMLOptionElement} */ (document.createElement("option"));
    emptyOption.value = (0xFF).toString();
    emptyOption.textContent = "UNBOUND";
    let elems = [emptyOption];

    let btnNames = []
    switch (commsMode) {
        case CommsMode.N64: 
            btnNames = N64Buttons;
            break;
        case CommsMode.Gamecube:
            //btnNames = ;
            break;
    }
    
    for (let i = 0; i < 32; i++) {
        if (btnNames[i]) {
            let optionElem = /** @type {HTMLOptionElement} */ (document.createElement("option"));
            optionElem.value = (i+1).toString();
            optionElem.textContent = btnNames[i];
            elems.push(optionElem);
        }
    }
    return elems;
}

export function placeRemapping(data) {
    remapDiv.replaceChildren();

    const commsMode = data.getUint8(1);

    for (let i = 0; i < 32; i++) {
        if (buttonNames[i]) {
            let inputElem = document.createElement("select");
            const optElems = getRemappingOptionsElems(commsMode);
            const bind = data.getUint8(i + 2);
            optElems.forEach(optElem => {
                if ((Number(optElem.value) == bind)
                ||  ((bind == 0) && Number(optElem.value) == (i+1))) {
                    optElem.selected = true;
                }
                // TODO; this could be problematic
                inputElem.appendChild(optElem);
            });
            inputElem.name = buttonNames[i];
            inputElem.onchange = setBinding;
            let spanElem = document.createElement("span");
            spanElem.textContent = buttonNames[i];
            remapDiv.appendChild(spanElem);
            remapDiv.appendChild(inputElem);
            remapDiv.appendChild(document.createElement("br")); 
        }
    }
}