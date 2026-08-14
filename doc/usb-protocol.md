# USB personalities and Zenith vendor protocol

## Default: Switch Pro + configuration

The default personality uses Nintendo's wired Pro Controller identity
(`057E:2009`) on interface 0. Interface 1 is vendor-specific WinUSB/WebUSB with
bulk OUT endpoint 2 and bulk IN endpoint 2. Commands and replies are fixed at a
maximum of 64 bytes; byte 0 is the command ID.

The existing calibration and settings commands are unchanged. Added commands:

| Command | Direction | Payload |
| --- | --- | --- |
| `09` | host → device | Set operating mode in byte 1 (`0` N64, `1` GameCube, `2` XInput) |
| `A9` | host → device → host | Get operating mode; reply byte 1 is the mode |
| `AA` | host → device → host | Get live input; bytes 1–4 are the exact corrected N64 poll report, byte 5 is the calibration flag, and little-endian float32 raw X/Y are at bytes 8/12 |

Button-map target IDs are `0` N64, `1` GameCube, `2` XInput, and `3` Switch.
Map entries remain physical-to-logical and use one-based destination bit
indices; `0` means identity and `FF` means unbound.

## XInput

XInput mode uses the Xbox 360 wired identity (`045E:028E`) and a single XInput
interface. It does not expose WebUSB. Hold physical A (`b1`) while plugging in
to restore and persist operating mode 0, which re-enables Switch Pro + WebUSB.
Hold physical B (`b2`) while plugging in to select and persist XInput mode.
If both are held, A takes priority.
