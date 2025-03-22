/*jshint esversion: 6 */
// @ts-check

import { IntToFloat32, swap32 } from "./utils.js";
import { calStep } from "./zenith_calib.js";

let xValSpan = /** @type {HTMLSpanElement} */ (document.getElementById("x-val"));
let yValSpan = /** @type {HTMLSpanElement} */ (document.getElementById("y-val"));
let dbgSpan = /** @type {HTMLSpanElement} */ (document.getElementById("dbg-text"));
let dbgCheckbox = /** @type {HTMLInputElement} */ (document.getElementById("debug-reporting-checked"));
let canvas = /** @type {HTMLCanvasElement} */ (document.getElementById("controller-display"));
let ctx = /** @type {CanvasRenderingContext2D} */ (canvas.getContext("2d"));

// parameters for drawing input display
const stickCircRad = 6;
export const stickCard = 85;
export const stickDiag = 70;
const stickMax = 127;
const inputDisplayScale = 0.9;

let controllerState = {
    ax: 0,
    ay: 0,
    fx: 0.0,
    fy: 0.0,
}

let csvData = ["x,y,rx,ry"];
const logCsv = false;

const download = async function () {
  
    // Creating a Blob for having a csv file format 
    // and passing the data with type
    const blob = new Blob([csvData.join("\n")], { type: 'text/csv' });
  
    // Creating an object for downloading url
    const url = window.URL.createObjectURL(blob)
  
    // Creating an anchor(a) tag of HTML
    const a = document.createElement('a')
  
    // Passing the blob downloading url 
    a.setAttribute('href', url)
  
    // Setting the anchor tag attribute for downloading
    // and passing the download file name
    a.setAttribute('download', 'download.csv');
  
    // Performing a download with click
    a.click()
}

export function setAx(ax) {
    controllerState.ax = ax;
    xValSpan.textContent = ax;
}

export function setAy(ay) {
    controllerState.ay = ay;
    yValSpan.textContent = ay;
}

export async function updateDbgReporting() {
    dbgSpan.style.visibility = dbgCheckbox.checked ? "visible" : "hidden";
}

export function updateInputDisplay(data) {
    if (calStep == -1) {
        setAx(data.getInt8(0));
        setAy(-data.getInt8(1));
    }
    if (logCsv) { csvData.push(`${data.getInt8(0)},${data.getInt8(1)}` + ',')}
 
    const rx = data.getUint32(11);
    const ry = data.getUint32(15);
    const fx = IntToFloat32(swap32(rx));
    const fy = IntToFloat32(swap32(ry));
    controllerState.fx = fx;
    controllerState.fy = fy;
    if (dbgCheckbox.checked)
        dbgSpan.textContent = `raw: x: ${fx.toFixed(4)}; y: ${fy.toFixed(4)}`;

    updateCanvas();
}


export function updateCanvas() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.save();
    ctx.translate(canvas.width/2,canvas.height/2);
    ctx.scale(inputDisplayScale, inputDisplayScale);

    ctx.lineWidth = 3;
    ctx.fillStyle ="white";

    // asumes canvas.width = canvas.height
    const sd = stickDiag/stickMax * canvas.width/2;
    const sc = stickCard/stickMax * canvas.width/2;
    ctx.beginPath();
    ctx.moveTo(0, sc);
    ctx.lineTo(sd, sd);
    ctx.lineTo(sc, 0);
    ctx.lineTo(sd, -sd);
    ctx.lineTo(0, -sc);
    ctx.lineTo(-sd, -sd);
    ctx.lineTo(-sc, 0);
    ctx.lineTo(-sd, sd);
    ctx.closePath();
    ctx.stroke();

    ctx.beginPath();
    const sx = controllerState.ax/stickMax * canvas.width/2;
    const sy = -controllerState.ay/stickMax * canvas.width/2;
    ctx.arc(sx, sy, stickCircRad, 0, 2*Math.PI);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    if (dbgCheckbox.checked) {
        ctx.fillStyle = "green";
        ctx.beginPath();
        const fx = controllerState.fx * canvas.width/2;
        const fy = -controllerState.fy * canvas.width/2;
        ctx.arc(fx, fy, stickCircRad/1.5, 0, 2*Math.PI);
        ctx.closePath();
        ctx.fill();
    }

    ctx.beginPath();
    ctx.lineWidth = 1;
    ctx.moveTo(-stickMax,stickMax);
    ctx.lineTo(stickMax,stickMax);
    ctx.lineTo(stickMax,-stickMax);
    ctx.lineTo(-stickMax,-stickMax);
    ctx.closePath();
    ctx.stroke();
    ctx.restore();
}