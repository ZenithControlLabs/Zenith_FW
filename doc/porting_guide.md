# Zenith Porting Guide

This (incomplete) document aims to capture some tips on starting a new project using the Zenith library.

## Controller reading and calibration

Zenith does all stick handling on core1, including both the hardware reading (userspace) and signal processing (in-library, but possibly might have userspace components at a later date). Core 0, which does all outgoing comms, reads from the global state set by core1.

TODO: add a description for each of the relevant callback functions