import { connect, saveSettings, resetSettings, setOperatingMode, updateFw } from "../../lib/modules/cntlr.js";
import { prevStep, nextStep, startCalib } from "../../lib/modules/zenith_calib.js";
import { setRemapMode } from "../../lib/modules/zenith_remap.js";
import { updateNotchPoint, updateMagThresh, updateGateLimiter, updateLpfCutoff } from "../../lib/modules/zenith_notch.js";
import { updateDbgReporting } from "../../lib/modules/zenith_input.js";

// functions exported to the HTML world
window._fns = {
    connect,
    prevStep,
    nextStep,
    startCalib,
    saveSettings,
    resetSettings,
    setRemapMode,
    setOperatingMode,
    updateNotchPoint,
    updateDbgReporting,
    updateMagThresh,
    updateGateLimiter,
    updateLpfCutoff,
    updateFw
};

///////////
// INIT //
/////////

document.getElementById("app-title").textContent = `${productName} Config Tool`
