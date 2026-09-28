#pragma once
#include "stubs/Arduino.h"
#include "stubs/avr/wdt.h"
#include <math.h>
#define F_CPU 16000000UL
#define ISR(x) void x()
inline volatile uint8_t PORTA=0,PORTC=0,PORTD=0,DDRA=0,DDRC=0;
inline volatile uint16_t TCCR1A=0,TCCR1B=0,OCR1A=0,TCNT1=0,TIMSK1=0;
constexpr int WGM12=0,CS11=1,OCIE1A=2;
inline void noInterrupts() {}
inline void interrupts() {}
