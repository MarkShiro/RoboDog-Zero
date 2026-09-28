#include <assert.h>
#include <stdio.h>
#include "stubs/TMCStepper.h"
#include "stubs/avr/wdt.h"
// Host test suppresses AVR section/naked attributes, not firmware logic.
#define __attribute__(x)
#include "../firmware/RoboDogMotorMap/RoboDogMotorMap.ino"
#undef __attribute__
void send(const char* s) { Serial.feed(s); loop(); }
void advance() { clockUs+=6000; loop(); }
int main() {
  analogs[A0]=873; analogs[A1]=682; analogs[A2]=682;
  setup(); pins[RUN_SWITCH]=LOW;
  assert(!powered && pins[ENABLE]==HIGH);
  send("J +50\n"); assert(!moving && pins[ENABLE]==HIGH);
  send("SCAN\n"); assert(configured && !moving && pins[ENABLE]==HIGH);
  send("J2\nARM\n"); assert(selected==1 && armed && pins[ENABLE]==HIGH);
  send("J +50\n"); assert(moving);
  for(int i=0;i<100;++i) advance();
  assert(!moving && !armed && pins[ENABLE]==HIGH && pulses[23]==50 && position[1]==50);
  for(int p=22;p<=29;++p) if(p!=23) assert(pulses[p]==0);
  send("J +50\n"); assert(!moving && pulses[23]==50);
  send("ARM\nJ -50\n"); for(int i=0;i<100;++i) advance();
  assert(position[1]==0 && budget[1]==100);
  send("ARM\n"); clockUs+=10001000; loop(); assert(!armed);
  send("ARM\nJ +50\n"); pins[42]=LOW; advance();
  assert(!powered && !moving && pulses[23]==100);
  pins[42]=HIGH;
  send("SCAN\nARM\nJ +50\n"); send("!");
  assert(!powered && pins[ENABLE]==HIGH);
  send("SCAN\n"); driver[3].online=false; send("ARM\n");
  assert(!powered && !armed); driver[3].online=true;
  send("SCAN\nARM\nJ +50\n"); pins[RUN_SWITCH]=HIGH; advance();
  assert(!powered && pins[ENABLE]==HIGH); pins[RUN_SWITCH]=LOW;
  send("SCAN\n"); clockUs+=30001000; loop(); assert(!powered);
  send("SCAN\nARM\n"); send("J +50garbage\n"); assert(!moving && !armed);
  send("ARM\n"); send("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n"); assert(!powered);
  puts("PASS: startup, UART fault, explicit arming, selected-only pulses, stop, limits, timeouts, malformed commands");
}
