#pragma once
#include <stdint.h>
// Software counters track commanded steps only. They are NOT joint encoders.
// 3200 pulses/rev * 12 / 360 = 106.6667 pulses per output degree.
constexpr int16_t MAX_JOG_STEPS=100; // <1 degree per command.
constexpr int16_t MAX_NET_STEPS=500; // <4.7 degrees from boot command origin.
constexpr uint16_t MAX_TOTAL_STEPS=1000; // Includes reverse moves; reset only at reboot.
constexpr bool benchJogAllowed(int16_t pos,uint16_t used,int16_t n) {
  return n!=0 && n>=-MAX_JOG_STEPS && n<=MAX_JOG_STEPS &&
    (int32_t)pos+n>=-MAX_NET_STEPS && (int32_t)pos+n<=MAX_NET_STEPS &&
    (uint32_t)used+(n<0?-(int32_t)n:n)<=MAX_TOTAL_STEPS;
}
static_assert(benchJogAllowed(0,0,50),"small jog");
static_assert(!benchJogAllowed(0,0,0),"no zero jog");
static_assert(!benchJogAllowed(0,0,101),"single jog cap");
static_assert(!benchJogAllowed(490,100,50),"positive bound");
static_assert(!benchJogAllowed(-490,100,-50),"negative bound");
static_assert(!benchJogAllowed(0,980,-50),"absolute pulse budget");
