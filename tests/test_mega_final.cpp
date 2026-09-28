#include <assert.h>
#include <stdio.h>
#include "mega_host.h"
#include "../firmware/RoboDogMegaFinal/steppers.cpp"
#include "../firmware/RoboDogMegaFinal/link.cpp"
#include "../firmware/RoboDogMegaFinal/power.cpp"
#include "../firmware/RoboDogMegaFinal/motion.cpp"
#include "../firmware/RoboDogMegaFinal/imu.h"
#include "../firmware/RoboDogMegaFinal/leds.h"
bool Imu::begin() { return false; }
void Imu::update() {}
void Leds::begin() {}
void Leds::set(LedMode) {}
void Leds::update(bool) {}
#define __attribute__(x)
#include "../firmware/RoboDogMegaFinal/RoboDogMegaFinal.ino"
#undef __attribute__
void packet(const char* body) {
  char line[100]; snprintf(line,sizeof(line),"%s*%02X\n",body,wireCrc(body,strlen(body))); Serial3.feed(line);
}
void resetLink() { link=Link(); Serial3.head=Serial3.tail=0; clockUs=100000; }
int main() {
  assert(wireCrc("123456789",9)==0xF4); // independent CRC-8/SMBUS check vector
  assert(!calibrationReady());
  WireControl c;
  const char* invalid[]={"", "C", "C,1,0000,7000,2", "C,1,0000,7000,2,0,1",
    "C,,0,1,2,0", "C,65536,0,1,2,0", "C,1,0400,1,2,0", "C,1,0,1,6,0",
    "C,1,0,-1,2,0", "C,1,0,1,2,0x", "C,1,0,1,2,256"};
  for(auto p:invalid) assert(!parseControlBody(p,c));
  assert(parseControlBody("C,65535,03FF,65535,5,255",c));
  assert(sequenceNewer(0,65535) && !sequenceNewer(65535,0) && !sequenceNewer(7,7));
  resetLink(); packet("C,65535,0000,7000,2,0"); link.poll(); assert(link.fresh());
  packet("A,65535"); link.poll(); assert(link.takeArmRequest());
  packet("A,65535"); link.poll(); assert(!link.takeArmRequest());
  clockUs+=400000; packet("C,65535,0000,7000,2,0"); link.poll();
  clockUs+=100000; assert(!link.fresh());
  packet("C,0,0000,7000,2,0"); link.poll(); assert(link.fresh());
  packet("A,0"); packet("C,1,0001,7000,2,0"); link.poll(); assert(!link.takeArmRequest());
  packet("C,0,0200,7000,2,0"); link.poll(); assert(link.takeStopRequest());
  assert(link.ctrl().buttons==1); // old STOP acts without becoming a fresh command
  resetLink(); Serial3.feed("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
  packet("C,3,0000,7000,2,0"); link.poll(); link.poll(); assert(!link.fresh());
  packet("C,4,0000,7000,2,0"); link.poll(); assert(link.fresh());
  packet("K,0"); link.poll(); assert(!link.piAcked());

  Steppers::begin(); Steppers::setSpeed(0,1000); clockUs+=10000; Steppers::tick();
  for(int i=0;i<100;++i) TIMER1_COMPA_vect();
  assert(Steppers::position(0)==0 && PORTA==0);
  Steppers::enable(true); Steppers::setSpeed(0,1000); clockUs+=100000; Steppers::tick();
  for(int i=0;i<500;++i) TIMER1_COMPA_vect();
  assert(Steppers::position(0)>0);
  const int32_t pos=Steppers::position(0);
  Steppers::enable(false); Steppers::setSpeed(0,1000); Steppers::tick();
  for(int i=0;i<100;++i) TIMER1_COMPA_vect();
  assert(Steppers::position(0)==pos && PORTA==0);
  Steppers::enable(true);
  for(int i=0;i<4001;++i) TIMER1_COMPA_vect();
  assert(!Steppers::enabled() && Steppers::timingFault() && (PORTD&128));
  Steppers::zero(0); Steppers::enable(true); Steppers::setBounds(true);
  Steppers::setSpeed(0,1000); clockUs+=100000; Steppers::tick();
  for(int i=0;i<1000;++i) TIMER1_COMPA_vect();
  assert(!Steppers::enabled() && Steppers::position(0)==0 && PORTA==0);
  Steppers::setBounds(false);
  g_drv[0].toff(4); g_drv[0].I_scale_analog(false); g_drv[0].mstep_reg_select(true);
  assert(Steppers::checkDriver(0));
  g_drv[0].fault=1; assert(!Steppers::checkDriver(0)); g_drv[0].fault=0;
  g_drv[0].online=false; assert(!Steppers::checkDriver(0)); g_drv[0].online=true;
  Steppers::enable(true); Steppers::renewLease(10);
  for(int i=0;i<81;++i) TIMER1_COMPA_vect();
  assert(!Steppers::enabled()); // command deadline shorter than the loop watchdog

  resetLink(); analogs[A0]=873; analogs[A1]=682; analogs[A2]=682;
  setup(); pins[PIN_PWR_REQ]=LOW; clockUs+=100000; loop();
  assert(!armed && !power.motorsPowered() && pins[PIN_DRV_EN]==HIGH);
  packet("C,10,0000,7000,2,0"); packet("A,10"); loop();
  assert(!armed && !power.motorsPowered()); // calibration lock enforced end-to-end
  Serial.feed("ARM HOME\n"); loop(); assert(!armed && !power.motorsPowered());

  // A reconnect packet may not mask the timeout that has already elapsed.
  armed=true; power.motorsPower(true); Steppers::enable(true);
  clockUs+=600000; packet("C,11,0000,7000,2,0"); loop();
  assert(!armed && !power.motorsPowered() && !motion.homed());
  // Physical shutdown request never cuts KEEP_ALIVE merely on an old K ACK.
  pins[PIN_PWR_REQ]=HIGH; clockUs+=100000; loop(); assert(state==ST_SHUTDOWN);
  packet("K,0"); clockUs+=70000000; loop();
  assert(pins[PIN_KEEP_ALIVE]==HIGH && !armed);
  puts("PASS MegaFinal: strict framing, sequence expiry, ARM lock, stale STOP, disabled ISR, lease, bounds, reconnect, shutdown");
}
