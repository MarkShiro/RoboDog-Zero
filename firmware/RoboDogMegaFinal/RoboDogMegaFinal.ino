// RoboDog Mega integration firmware, revision 2026-09-27.
// Default: calibration lock, telemetry only. Read README.md before powering drives.
#include <avr/wdt.h>
#include "config.h"
#include "steppers.h"
#include "power.h"
#include "imu.h"
#include "leds.h"
#include "link.h"
#include "motion.h"

uint8_t resetReason __attribute__((section(".noinit")));
void earlyWatchdogOff() __attribute__((naked,section(".init3")));
void earlyWatchdogOff() { resetReason=MCUSR; MCUSR=0; wdt_disable(); }

// Preserve remote T state compatibility (0..5).
enum State : uint8_t { ST_BOOT=0,ST_IDLE,ST_RUN,ST_WARN,ST_ESTOP,ST_SHUTDOWN };
Power power; Imu imu; Leds leds; Link link; Motion motion;
State state=ST_IDLE;
bool armed=false, driversOk=false, supplyStarting=false;
uint32_t shutdownStart=0,lastShutdownTx=0,lastDriverCheck=0;
uint8_t nextDriver=0;
char usbLine[32]; uint8_t usbLen=0; bool usbOverflow=false;

void disarm(const __FlashStringHelper* reason) {
  Steppers::enable(false); // hardware gate before other work
  motion.halt(); power.motorsPower(false); driversOk=false; armed=false;
  link.cancelArm();
  if(state!=ST_SHUTDOWN) state=ST_ESTOP;
  leds.set(LED_ESTOP);
  Serial.print(F("DISARM: ")); Serial.println(reason);
}
void enterShutdown() {
  if(state==ST_SHUTDOWN) return;
  disarm(F("SHUTDOWN")); state=ST_SHUTDOWN;
  shutdownStart=millis(); lastShutdownTx=millis()-2000UL;
  leds.set(LED_SHUTDOWN);
  Serial.println(F("Pi shutdown requested. Auto power release is disabled by default."));
}
// Called between driver transactions while outputs remain disabled.
bool serviceArming() {
  power.update();
  const bool expired=!link.fresh(); // check BEFORE accepting a reconnect baseline
  link.poll();
  if(expired || !link.fresh() || link.takeStopRequest() || link.ctrl().buttons ||
     !power.runRequested() || power.estopPressed() || !power.senseSane() ||
     power.vbus()<=VBUS_DOWN_MV || (!supplyStarting && !power.railsSane())) return false;
  wdt_reset(); return true;
}
void requestArm() {
  if(armed || state==ST_SHUTDOWN) return;
  if(!calibrationReady()) { Serial.println(F("ARM REFUSED: fill and verify calibration.h")); return; }
  if(!link.fresh() || link.ctrl().buttons || !power.runRequested() ||
     power.estopPressed() || !power.senseSane() || power.vbus()<=VBUS_DOWN_MV) {
    disarm(F("ARM interlock/control")); return;
  }
  Steppers::enable(false);
  power.motorsPower(true); supplyStarting=true;
  const uint32_t started=millis();
  while(millis()-started<500UL) {
    if(!serviceArming()) { supplyStarting=false; disarm(F("supply startup")); return; }
    delay(5);
  }
  supplyStarting=false;
  driversOk=Steppers::setupDrivers(serviceArming);
  if(!driversOk || !serviceArming()) { disarm(F("driver/config/rail failure")); return; }
  Steppers::enable(true); armed=true; state=STARTUP_ZERO_MODE?ST_IDLE:ST_BOOT;
  Steppers::renewLease(link.remainingMs());
  if(STARTUP_ZERO_MODE) motion.startRelativeZero();
  else motion.startHoming();
  lastDriverCheck=millis(); nextDriver=0;
  if(motion.fault()) disarm(F("home refused"));
  else if(STARTUP_ZERO_MODE) Serial.println(F("ARMED: startup position = relative zero; no automatic motion"));
  else Serial.println(F("ARMED: bounded HOME then calibrated STAND"));
}
void printStatus() {
  Serial.print(F("state=")); Serial.print((uint8_t)state);
  Serial.print(F(" calibrated=")); Serial.print(calibrationReady());
  Serial.print(F(" armed=")); Serial.print(armed);
  Serial.print(F(" fresh=")); Serial.print(link.fresh());
  Serial.print(F(" vbus=")); Serial.print(power.vbus());
  Serial.print(F(" railA=")); Serial.print(power.v20f());
  Serial.print(F(" railB=")); Serial.println(power.v20r());
  Serial.print(F("positions J1..J8: "));
  for(uint8_t i=0;i<MOTOR_COUNT;++i) { Serial.print(Steppers::position(i)); Serial.print(' '); }
  Serial.println();
}
void pollUsb() {
  uint8_t budget=48;
  while(budget-- && Serial.available()) {
    const char c=(char)Serial.read();
    if(c=='!') { disarm(F("USB STOP")); usbLen=0; usbOverflow=false; continue; }
    if(c=='\r') continue;
    if(c=='\n') {
      if(!usbOverflow && usbLen) {
        usbLine[usbLen]=0;
        if(!strcmp(usbLine,"ARM HOME") && !STARTUP_ZERO_MODE) requestArm();
        else if(!strcmp(usbLine,"ARM ZERO") && STARTUP_ZERO_MODE) requestArm();
        else if(!strcmp(usbLine,"OFF")) disarm(F("USB OFF"));
        else if(!strcmp(usbLine,"STATUS")) printStatus();
        else if(!strcmp(usbLine,"SHUTDOWN")) enterShutdown();
        else if(!strcmp(usbLine,"STAND") && armed && motion.homed()) motion.setPose(POSE_STAND);
        else if(STARTUP_ZERO_MODE) Serial.println(F("Commands: STATUS, ARM ZERO, STAND, OFF, SHUTDOWN, !"));
        else Serial.println(F("Commands: STATUS, ARM HOME, STAND, OFF, SHUTDOWN, !"));
      }
      usbLen=0; usbOverflow=false;
    } else if(!usbOverflow) {
      if(c<32 || c>126 || usbLen>=sizeof(usbLine)-1) { usbOverflow=true; usbLen=0; }
      else usbLine[usbLen++]=c;
    }
  }
}
uint16_t buildFlags() {
  uint16_t flags=0;
  if(driversOk) flags|=FLG_DRIVERS_OK;
  if(imu.ok()) flags|=FLG_IMU_OK;
  if(motion.homed()) flags|=FLG_HOMED;
  if(STARTUP_ZERO_MODE) flags|=FLG_RELATIVE_ZERO;
  if(power.motorsPowered()) flags|=FLG_MOT_POWER;
  if(power.braking()) flags|=FLG_BRAKE;
  if(power.senseSane() && power.vbus()<=VBUS_WARN_MV) flags|=FLG_LOWBAT;
  if(!link.fresh()) flags|=FLG_NO_CTRL;
  if(!power.senseSane() || (power.motorsPowered() && !power.railsSane())) flags|=FLG_SENSE_BAD;
  return flags;
}
void setup() {
  digitalWrite(PIN_DRV_EN,HIGH); pinMode(PIN_DRV_EN,OUTPUT);
  power.begin(); Steppers::begin(); motion.begin(); link.begin();
  Serial.begin(115200);
  wdt_enable(WDTO_4S);
  leds.begin(); leds.set(LED_WARN); imu.begin(); power.update();
  Serial.println(F("RoboDogMegaFinal: DISARMED. Gear 12, 600mA RMS, 1/16."));
  Serial.println(F("Default calibration lock; no automatic homing. STATUS for diagnostics."));
}
void loop() {
  wdt_reset();
  power.update();
  // Freshness must be checked before poll can accept the first reconnect packet.
  if(armed && !link.fresh()) disarm(F("control timeout"));
  if(armed && (power.estopPressed() || !power.senseSane() || !power.railsSane() ||
               Steppers::timingFault())) disarm(F("ESTOP/voltage/step-limit/loop-stall"));
  if(state!=ST_SHUTDOWN && (!power.runRequested() ||
      (power.senseSane() && power.vbus()<=VBUS_DOWN_MV))) enterShutdown();

  link.poll();
  const bool stop=link.takeStopRequest() || (link.fresh() && (link.ctrl().buttons & CMD_STOP));
  if(stop && (armed || power.motorsPowered())) disarm(F("remote STOP"));
  if(stop) link.cancelArm();
  if(link.takeArmRequest() && !stop) requestArm();
  pollUsb();

  if(armed && motion.homing() && !serviceArming()) {
    // During HOME only neutral heartbeat is accepted, never queued walking commands.
    disarm(F("HOME interrupted"));
  }
  // serviceArming is intentionally NOT used for ordinary motion (buttons are expected).
  if(armed && (!link.fresh() || (link.ctrl().buttons & CMD_STOP))) disarm(F("control lost/STOP"));
  if(armed && millis()-lastDriverCheck>=250UL) {
    lastDriverCheck=millis();
    if(!Steppers::checkDriver(nextDriver)) disarm(F("driver runtime fault"));
    nextDriver=(nextDriver+1)%MOTOR_COUNT;
  }
  if(armed && Steppers::timingFault()) disarm(F("step-limit/loop-stall"));
  if(armed && !link.fresh()) disarm(F("control expired during driver check"));
  if(armed) {
    if(motion.homed()) {
      const uint16_t b=link.ctrl().buttons;
      const int8_t fwd=((b&CMD_FWD)?1:0)-((b&CMD_BACK)?1:0);
      const int8_t turn=((b&CMD_RIGHT)?1:0)-((b&CMD_LEFT)?1:0);
      if(b&CMD_STAND) motion.setPose(POSE_SIT);
      else motion.walk(fwd,turn,link.ctrl().speed);
      state=(fwd || turn) && ENABLE_EXPERIMENTAL_TROT?ST_RUN:ST_IDLE;
      if(power.vbus()<=VBUS_WARN_MV) state=ST_WARN;
    }
    motion.update();
    if(motion.fault()) disarm(F("motion/home/limit fault"));
    else { Steppers::tick(); Steppers::renewLease(link.remainingMs()); }
  }
  if(state==ST_SHUTDOWN) {
    if(millis()-lastShutdownTx>=2000UL) { lastShutdownTx=millis(); link.sendShutdown(); }
    // K is ignored. Optional measured fallback delay is never shortened by ACK.
    if(AUTOMATIC_POWER_RELEASE && millis()-shutdownStart>=PI_SHUTDOWN_MS+POWER_OFF_MS) {
      power.releasePower(); while(true) wdt_reset();
    }
  }
  imu.update();
  power.fan(power.motorsPowered()?255:0);
  leds.update(motion.moving());
  link.sendTelemetry((uint8_t)state,power.vbus(),buildFlags());
  if(imu.ok()) link.sendImu(imu.rollDeg(),imu.pitchDeg(),imu.tempC(),buildFlags());
}
