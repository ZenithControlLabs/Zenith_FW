* Modules define their own temporary global state (i.e. step in calibration routine), and do NOT expose it
* If it needs to be saved in the onboard QSPI flash, it needs to go in the zenith_settings struct
* Don't put it in a header unless it needs to be exposed.
* If it's a global variable, prefix it with "_".
* constants are CAPITALIZED (don't need the prefix for a global variable)
* Only declare PUBLIC functions in the header?
* write include guards like this: `ZENITH_MYFILE_H` for myfile.h
