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
* [NS-LIB-HID](https://github.com/HandHeldLegend/NS-LIB-HID) and
  [HHL-TINYUSB-DRIVERS](https://github.com/HandHeldLegend/HHL-TINYUSB-DRIVERS)
    * Referenced for the Switch Pro protocol/descriptors and TinyUSB XInput
      class driver design. NS-LIB-HID is distributed under CC BY-NC 4.0.

## USB modes

The default USB personality is a wired Nintendo Switch Pro Controller
(VID `057E`, PID `2009`) with a separate WebUSB/WinUSB vendor interface for
configuration and corrected N64 input. XInput mode enumerates as an Xbox 360
controller and intentionally omits that vendor interface.

At plug-in, holding physical **A** (`b1`) selects and saves the default N64 wired
+ Switch Pro USB mode; holding physical **B** (`b2`) selects and saves XInput
mode. A takes priority if both are held, so the WebUSB configurator can always
be recovered.

The `project64-plugin` directory contains source and x86/x64 builds of the
Project64 raw-input plugin.
