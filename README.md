# Zenith Library

A fork of [HOJA-LIB](https://github.com/HandHeldLegend/HOJA-LIB-RP2040/tree/master) with an emphasis on N64 controllers and portability for different hardware platforms.

## Credits

Outside of HOJA-LIB, below is a list of the various software containing code incorporated into Zenith, and where it is used.

* [PhobGCC-SW](https://github.com/PhobGCC/PhobGCC-SW)
    * Notch remapping/linearization algorithm.
* [joybus-pio](https://github.com/JonnyHaystack/joybus-pio)
    * Used PIO side of joybus-pio, C side is custom.
* [HayBox](https://github.com/JonnyHaystack/HayBox)
    * Referenced for USB comms
* [polyfit](https://github.com/henryfo/polyfit)
    * Linear curve fitting backend, modified to work in embedded