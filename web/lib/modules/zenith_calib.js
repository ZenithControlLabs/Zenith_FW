/*jshint esversion: 6 */
// @ts-check

import { WebUSBCmdMap, setSaveIndicator, writeUSBCmd } from "./cntlr.js";
import { setAx, setAy, stickCard, stickDiag, updateCanvas } from "./zenith_input.js";

let standardDiv = /** @type {HTMLDivElement} */ (document.getElementById("standard-div"));
let calDiv = /** @type {HTMLDivElement} */ (document.getElementById("cal-div"));
let calStepSpan = /** @type {HTMLSpanElement} */ (document.getElementById("cal-step"));

const calPoints = [[stickCard,0],
                   [stickDiag,stickDiag],
                   [0,stickCard],
                   [-stickDiag,stickDiag],
                   [-stickCard,0],
                   [-stickDiag,-stickDiag],
                   [0,-stickCard],
                   [stickDiag,-stickDiag]];
const numCalSteps = 16;

export let calStep = -1;


function setCalStep(calStep_in) {
    calStep = calStep_in;
    calStepSpan.textContent = calStep.toString();
    if (calStep % 2 == 0) {
        setAx(calPoints[(calStep-1) >> 1][0])
        setAy(calPoints[(calStep-1) >> 1][1])
    } else {
        setAx(0);
        setAy(0);
    }
}

export async function startCalib() {
    setCalStep(1);
    calDiv.style.display = "block";
    standardDiv.style.display = "none";

    try
    {
        await writeUSBCmd(WebUSBCmdMap.CALIBRATION_START);
    }
    catch (e) {
        console.error(e.message);
    }
    
}

export async function nextStep() {
    if ((calStep+1) > numCalSteps) {
        setCalStep(-1);
        calDiv.style.display = "none";
        standardDiv.style.display = "flex";
        setSaveIndicator();
    } else {
        setCalStep(calStep+1);
    }
    updateCanvas();

    await writeUSBCmd(WebUSBCmdMap.CALIBRATION_ADVANCE);
}

export async function prevStep() {
    if (calStep == 1) return;
    setCalStep(calStep-1);
    updateCanvas();

    await writeUSBCmd(WebUSBCmdMap.CALIBRATION_UNDO);
}