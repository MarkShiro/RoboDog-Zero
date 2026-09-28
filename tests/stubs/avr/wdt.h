#pragma once
#define WDTO_2S 0
#define WDTO_4S 1
inline void wdt_disable() {}
inline void wdt_enable(int) {}
inline void wdt_reset() {}
inline unsigned char MCUSR=0;
