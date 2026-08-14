/*jshint esversion: 8 */
// @ts-check

import { WebUSBCmdMap, setSaveIndicator, writeUSBData } from "./cntlr.js";

const remapDiv = /** @type {HTMLDivElement} */ (document.getElementById("bindings-fill-in"));

export const RemapMode = { N64: 0, GameCube: 1, XInput: 2, Switch: 3 };
export const CommsMode = RemapMode;

const logicalNames = {
    [RemapMode.N64]: ["A", "B", "C Up", "C Down", "C Left", "C Right", "Start", "L", "R", "Z", "D Down", "D Left", "D Right", "D Up"],
    [RemapMode.GameCube]: ["A", "B", "X", "Y", "Start", "L", "R", "Z", "D Down", "D Left", "D Right", "D Up"],
    [RemapMode.Switch]: ["Y", "X", "B", "A", "", "", "R", "ZR", "Minus", "Plus", "R Stick", "L Stick", "Home", "Capture", "", "", "D Down", "D Up", "D Right", "D Left", "", "", "L", "ZL"],
    [RemapMode.XInput]: ["D Up", "D Down", "D Left", "D Right", "Start", "Back", "L Stick", "R Stick", "LB", "RB", "Guide", "", "A", "B", "X", "Y", "LT", "RT"]
};

export let _commsMode = RemapMode.Switch;

export async function setRemapMode(mode) {
    if (!Object.values(RemapMode).includes(mode)) return;
    _commsMode = mode;
    await writeUSBData(new Uint8Array([WebUSBCmdMap.REMAP_GET, mode]));
}

export const setCommsMode = setRemapMode;

async function setBinding(event) {
    const select = /** @type {HTMLSelectElement} */ (event.currentTarget);
    const source = Number(select.dataset.source);
    const destination = Number(select.value);
    await writeUSBData(new Uint8Array([
        WebUSBCmdMap.REMAP_SET, _commsMode, source, destination
    ]));
    setSaveIndicator();
}

function optionsFor(mode, source, binding) {
    const options = [{value: 0xFF, label: "Unbound"}];
    logicalNames[mode].forEach((label, index) => {
        if (label) options.push({value: index + 1, label});
    });
    return options.map(option => {
        const element = document.createElement("option");
        element.value = String(option.value);
        element.textContent = option.label;
        element.selected = option.value === binding ||
            (binding === 0 && option.value === source + 1);
        return element;
    });
}

export function placeRemapping(data) {
    const mode = data.getUint8(1);
    if (mode !== _commsMode) return;
    remapDiv.replaceChildren();
    for (let source = 0; source < 32; ++source) {
        if (!buttonNames[source]) continue;
        const row = document.createElement("label");
        row.className = "binding-row";
        const physical = document.createElement("span");
        physical.textContent = buttonNames[source];
        const arrow = document.createElement("span");
        arrow.className = "binding-arrow";
        arrow.textContent = "→";
        const select = document.createElement("select");
        select.className = "form-select";
        select.dataset.source = String(source);
        optionsFor(mode, source, data.getUint8(source + 2))
            .forEach(option => select.appendChild(option));
        select.addEventListener("change", setBinding);
        row.append(physical, arrow, select);
        remapDiv.appendChild(row);
    }
}
