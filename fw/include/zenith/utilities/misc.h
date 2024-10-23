#ifndef ZENITH_MISC_H
#define ZENITH_MISC_H

#ifdef DEBUG
#define debug_print(fmt, args...) printf(fmt, ##args)
#else
#define debug_print(fmt, args...)
#endif

#endif // ZENITH_MISC_H