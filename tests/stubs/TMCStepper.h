#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 54
#define A1 55
#define A2 56
#define A6 60
#define F(s) reinterpret_cast<const __FlashStringHelper*>(s)
class __FlashStringHelper;
inline int pins[70]={0};
inline int pulses[70]={0};
inline uint32_t clockUs=0;
inline int analogs[70]={0};
inline void pinMode(int pin,int mode) { if(mode==INPUT_PULLUP) pins[pin]=HIGH; }
inline int digitalRead(int p) { return pins[p]; }
inline void digitalWrite(int p,int v) { if(p>=22 && p<=29 && v && !pins[p]) ++pulses[p]; pins[p]=v; }
inline int analogRead(int p) { return analogs[p]; }
inline void analogWrite(int p,int v) { pins[p]=v; }
inline uint32_t micros() { return clockUs; }
inline uint32_t millis() { return clockUs/1000; }
inline void delay(unsigned n) { clockUs+=n*1000; }
inline void delayMicroseconds(unsigned n) { clockUs+=n; }
struct MockSerial {
 char input[1024]={}; unsigned head=0,tail=0;
 void begin(unsigned long) {}
 int available() { return tail-head; }
 int read() { return input[head++ % 1024]; }
 void feed(const char* s) { while(*s) input[tail++ % 1024]=*s++; }
 template<class T> void print(T) {}
 template<class T> void println(T) {}
 void println() {} void flush() {}
};
inline MockSerial Serial,Serial1,Serial2,Serial3;
class TMC2209Stepper {
 uint8_t counter=0;
 public:
 TMC2209Stepper(MockSerial*,float,int) {}
 void begin() {}
#define PROPERTY(type,name,initial) private: type v_##name=initial; public: type name() {return v_##name;} void name(type v) {v_##name=v;++counter;}
 PROPERTY(uint8_t,toff,0)
 PROPERTY(bool,I_scale_analog,true)
 PROPERTY(bool,internal_Rsense,false)
 PROPERTY(bool,mstep_reg_select,false)
 PROPERTY(uint16_t,microsteps,16)
#undef PROPERTY
 uint8_t IFCNT() { return counter; }
 uint32_t fault=0;
 bool online=true;
 uint32_t DRV_STATUS() { return fault; }
 uint32_t IOIN() { return online?0x21000000:0; }
 void pdn_disable(bool) {} void rms_current(uint16_t,float) {}
 void en_spreadCycle(bool) {} void blank_time(uint8_t) {}
 void hysteresis_start(uint8_t) {} void hysteresis_end(uint8_t) {}
 void intpol(bool) {} void TPOWERDOWN(uint8_t) {}
 void pwm_autoscale(bool) {} void iholddelay(uint8_t) {}
};
