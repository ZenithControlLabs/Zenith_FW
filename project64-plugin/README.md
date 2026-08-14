# Zenith Raw Input for Project64

This controller plugin reads the firmware-corrected N64 report directly from a
Zenith controller's private USB interface. It does not apply deadzones, curves,
or axis correction on the PC.

## Use

1. Keep the controller in its default **Switch Pro** USB mode. XInput mode does
   not expose the private configuration/raw-input interface.
2. Copy the DLL matching Project64's architecture from `dist/x86` or
   `dist/x64` into Project64's controller-plugin directory.
3. Select **Zenith Raw Input 1.0** as the input plugin.
4. Open the plugin configuration to change the controller's persistent N64
   button mapping.

If XInput mode was selected accidentally, unplug the controller, hold physical
**A**, and plug it back in. It will return to Switch Pro mode.
Holding physical **B** while plugging in selects and saves XInput mode instead.

The plugin and web configurator use the same private USB command interface.
Close the configurator before starting emulation if Windows does not allow both
applications to access that interface at the same time; the plugin displays an
error instead of silently returning no input when communication is unavailable.
