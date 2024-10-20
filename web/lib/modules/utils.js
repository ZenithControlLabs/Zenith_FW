/*jshint esversion: 6 */
// @ts-check

export function swap32(val) {
    return ((val & 0xFF) << 24)
           | ((val & 0xFF00) << 8)
           | ((val >> 8) & 0xFF00)
           | ((val >> 24) & 0xFF);
}

// https://gist.github.com/Jozo132/2c0fae763f5dc6635a6714bb741d152f 
export function IntToFloat32(int) {
    if (int > 0 || int < 0) {
        const sign = (int >>> 31) ? -1 : 1;
        let exp = (int >>> 23 & 0xff) - 127;
        const mantissa = ((int & 0x7fffff) + 0x800000).toString(2);
        let float32 = 0
        for (let i = 0; i < mantissa.length; i += 1) { float32 += parseInt(mantissa[i]) ? Math.pow(2, exp) : 0; exp-- }
        return float32 * sign;
    } else return 0;
}

export function sleep_ms(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}