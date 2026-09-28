// Mega 2560: supported-robot bench identification only. NO gait or homing.
// USB Serial 115200, newline. Orange Pi must be disconnected/unpowered.
// Shared EN D38; each unselected TMC is additionally disabled with TOFF=0.
// Read docs/Bringup-2026-09-25.md BEFORE connecting motor power.
#include <TMCStepper.h>
#include <avr/wdt.h>
#include "bench_limits.h"

// Disable a watchdog left over from a reset before setup/boot initialization.
uint8_t resetReason __attribute__((section(".noinit")));
void earlyWatchdogOff() __attribute__((naked,section(".init3")));
void earlyWatchdogOff() { resetReason=MCUSR; MCUSR=0; wdt_disable(); }

constexpr uint8_t KEEP_ALIVE=7, MOTOR_POWER=39, ENABLE=38, RUN_SWITCH=3;
constexpr uint8_t ESTOP=41, FAN=8, BRAKE_A=9, BRAKE_B=10;
// PDF does not show the separate D41 contact. Set true ONLY after wiring it.
constexpr bool HAS_ESTOP_SIGNAL=false;
constexpr float RSENSE=0.11f; // Confirm both sense resistors are marked R110.
constexpr uint16_t TEST_CURRENT_MA=600; // RMS, unloaded identification only.
constexpr uint8_t STEP_PIN[8]={22,23,24,25,26,27,28,29};
constexpr uint8_t DIR_PIN[8]={30,31,32,33,34,35,36,37};
constexpr uint8_t LIMIT_PIN[8]={42,43,44,45,46,47,48,49};
// J1..J4 on Serial1, J5..J8 on Serial2. Address = MS1 + 2*MS2.
// PDF: (MS1,MS2) = (0,0),(1,0),(0,1),(1,1) -> 0,1,2,3.
constexpr uint8_t ADDRESS[8]={0,1,2,3,0,1,2,3};
TMC2209Stepper driver[8]={
  TMC2209Stepper(&Serial1,RSENSE,0), TMC2209Stepper(&Serial1,RSENSE,1),
  TMC2209Stepper(&Serial1,RSENSE,2), TMC2209Stepper(&Serial1,RSENSE,3),
  TMC2209Stepper(&Serial2,RSENSE,0), TMC2209Stepper(&Serial2,RSENSE,1),
  TMC2209Stepper(&Serial2,RSENSE,2), TMC2209Stepper(&Serial2,RSENSE,3)
};
bool powered=false, configured=false, armed=false, moving=false;
uint8_t selected=0;
uint16_t remaining=0, budget[8]={0};
int16_t position[8]={0};
int8_t direction=1;
uint32_t armedAt=0, nextStep=0, lastSense=0, lastHost=0;
char command[40]; uint8_t commandLength=0; bool overflow=false;
bool brakeA=false, brakeB=false;

uint16_t voltage(uint8_t pin) {
  analogRead(pin); // Discard first sample after changing multiplexer.
  uint32_t sum=0; for(uint8_t i=0;i<4;++i) sum+=analogRead(pin);
  return sum*30000UL/(4UL*1023UL); // 5 V ADC, 100k/20k divider.
}
bool runAllowed() {
  return digitalRead(RUN_SWITCH)==LOW && (!HAS_ESTOP_SIGNAL || digitalRead(ESTOP)==HIGH);
}
bool allLimitsOpen() {
  for(uint8_t i=0;i<8;++i) if(digitalRead(LIMIT_PIN[i])==LOW) return false;
  return true;
}
void stopMotion() {
  // Hardware gate FIRST; UART can block, it must never delay this gate.
  digitalWrite(ENABLE,HIGH);
  for(uint8_t i=0;i<8;++i) digitalWrite(STEP_PIN[i],LOW);
  armed=false; moving=false; remaining=0;
}
void powerOff() {
  stopMotion(); digitalWrite(MOTOR_POWER,LOW); analogWrite(FAN,0);
  powered=false; configured=false;
}
void fail(const __FlashStringHelper* reason) {
  powerOff(); Serial.print(F("STOP: ")); Serial.println(reason);
}
void printStatus() {
  Serial.print(F("J")); Serial.print(selected+1);
  Serial.print(F(" bus=")); Serial.print(selected<4?'A':'B');
  Serial.print(F(" addr=")); Serial.print(ADDRESS[selected]);
  Serial.print(F(" STEP=")); Serial.print(STEP_PIN[selected]);
  Serial.print(F(" DIR=")); Serial.print(DIR_PIN[selected]);
  Serial.print(F(" commandedSteps=")); Serial.print(position[selected]);
  Serial.print(F(" pulseBudgetUsed=")); Serial.print(budget[selected]);
  Serial.print(F(" power=")); Serial.print(powered);
  Serial.print(F(" armed=")); Serial.println(armed);
  Serial.print(F("LIMIT D42..D49 (1=pressed): "));
  for(uint8_t i=0;i<8;++i) Serial.print(digitalRead(LIMIT_PIN[i])==LOW?'1':'0');
  Serial.println();
}
bool driverHealthy(uint8_t i) {
  const uint32_t status=driver[i].DRV_STATUS();
  // Communication zeros can be a valid unloaded state; IOIN version checks link.
  const uint8_t version=(uint8_t)(driver[i].IOIN()>>24);
  return version==0x21 && status!=0xFFFFFFFFUL &&
         !(status & ((1UL<<0)|(1UL<<1)|(1UL<<2)|(1UL<<3)|(1UL<<4)|(1UL<<5)));
}
void scan() {
  powerOff();
  if(!runAllowed()) { Serial.println(F("RUN switch off or ESTOP active")); return; }
  const uint16_t mv=voltage(A0);
  if(mv<24000 || mv>29500) { Serial.println(F("VBUS invalid/low: check A0 and pack")); return; }
  digitalWrite(MOTOR_POWER,HIGH); powered=true; analogWrite(FAN,255);
  delay(300); wdt_reset();
  bool ok=true;
  for(uint8_t i=0;i<8;++i) {
    wdt_reset();
    if(!runAllowed()) { fail(F("interlock during SCAN")); return; }
    driver[i].begin();
    driver[i].toff(0);
    driver[i].pdn_disable(true);
    driver[i].I_scale_analog(false);
    driver[i].internal_Rsense(false);
    driver[i].mstep_reg_select(true);
    driver[i].rms_current(TEST_CURRENT_MA,1.0f);
    driver[i].microsteps(16);
    driver[i].en_spreadCycle(true);
    driver[i].blank_time(24);
    driver[i].hysteresis_start(4); driver[i].hysteresis_end(1);
    driver[i].intpol(true);
    driver[i].TPOWERDOWN(20);
    const uint8_t before=driver[i].IFCNT();
    driver[i].toff(0);
    const uint8_t after=driver[i].IFCNT();
    const bool good=(uint8_t)(after-before)==1 && driver[i].toff()==0 &&
      !driver[i].I_scale_analog() && !driver[i].internal_Rsense() &&
      driver[i].mstep_reg_select() && driver[i].microsteps()==16 && driverHealthy(i);
    Serial.print(F("J")); Serial.print(i+1); Serial.print(F(" addr="));
    Serial.print(ADDRESS[i]); Serial.println(good?F(" OK"):F(" FAIL"));
    ok &=good;
  }
  if(!ok) { fail(F("UART/configuration: no motion permitted")); return; }
  configured=true; lastHost=millis();
  Serial.println(F("SCAN OK. Drivers DISABLED. Select J1..J8, ARM, then J +50."));
}
void arm() {
  stopMotion();
  if(!powered || !configured || !runAllowed() || !allLimitsOpen()) {
    Serial.println(F("ARM refused: SCAN, RUN and all limits open required")); return;
  }
  for(uint8_t i=0;i<8;++i) { wdt_reset(); driver[i].toff(0); }
  driver[selected].toff(4);
  for(uint8_t i=0;i<8;++i) {
    wdt_reset();
    if(driver[i].toff()!=(i==selected?4:0) || !driverHealthy(i)) {
      fail(F("driver readback/fault")); return;
    }
  }
  if(!runAllowed() || !allLimitsOpen()) { fail(F("interlock")); return; }
  // Still disabled: ARM alone must not energize the motor.
  armed=true; armedAt=millis(); Serial.println(F("ARMED for one jog, 10 seconds."));
}
void jog(int16_t steps) {
  if(!armed || !powered || !runAllowed() || !allLimitsOpen() ||
     !benchJogAllowed(position[selected],budget[selected],steps)) {
    stopMotion(); Serial.println(F("JOG refused: ARM/limit/budget")); return;
  }
  // Recheck after ARM: power loss may reset registers while waiting for J.
  for(uint8_t i=0;i<8;++i) {
    wdt_reset();
    if(driver[i].toff()!=(i==selected?4:0) || driver[i].I_scale_analog() ||
       !driver[i].mstep_reg_select() || driver[i].microsteps()!=16 || !driverHealthy(i)) {
      fail(F("driver changed since ARM")); return;
    }
  }
  if(!runAllowed() || !allLimitsOpen()) { fail(F("interlock before jog")); return; }
  if(voltage(A1)<17000 || voltage(A2)<17000) { fail(F("motor supply / A1,A2")); return; }
  direction=steps>0?1:-1; remaining=steps>0?steps:-steps;
  digitalWrite(DIR_PIN[selected],direction>0?HIGH:LOW);
  delayMicroseconds(10);
  digitalWrite(ENABLE,LOW);
  moving=true; armed=false;
  nextStep=micros()+50000UL; // Allow current to establish before first pulse.
}
void handleCommand() {
  lastHost=millis();
  if(!strcmp(command,"SCAN")) scan();
  else if(!strcmp(command,"ARM")) arm();
  else if(!strcmp(command,"OFF")) { powerOff(); Serial.println(F("POWER OFF")); }
  else if(!strcmp(command,"STATUS")) printStatus();
  else if(!strcmp(command,"QUIT")) {
    powerOff(); Serial.println(F("KEEP_ALIVE released; Orange Pi must be off."));
    Serial.flush(); digitalWrite(KEEP_ALIVE,LOW);
  }
  else if(command[0]=='J' && command[1]>='1' && command[1]<='8' && !command[2]) {
    stopMotion(); selected=command[1]-'1'; printStatus();
  }
  else if(command[0]=='J' && command[1]==' ' && (command[2]=='+' || command[2]=='-')) {
    char* end; long n=strtol(command+2,&end,10);
    if(end==command+3 || *end || n < -100 || n>100 || n==0) {
      stopMotion(); Serial.println(F("Use J +1..100 or J -1..100"));
    } else jog((int16_t)n);
  }
  else { stopMotion(); Serial.println(F("Commands: SCAN J1..J8 ARM J +50 J -50 STATUS OFF QUIT !")); }
}
void setup() {
  digitalWrite(ENABLE,HIGH); pinMode(ENABLE,OUTPUT);
  digitalWrite(MOTOR_POWER,LOW); pinMode(MOTOR_POWER,OUTPUT);
  digitalWrite(KEEP_ALIVE,HIGH); pinMode(KEEP_ALIVE,OUTPUT);
  pinMode(RUN_SWITCH,INPUT_PULLUP); pinMode(ESTOP,INPUT_PULLUP);
  pinMode(FAN,OUTPUT); analogWrite(FAN,0);
  pinMode(BRAKE_A,OUTPUT); pinMode(BRAKE_B,OUTPUT);
  digitalWrite(BRAKE_A,LOW); digitalWrite(BRAKE_B,LOW);
  for(uint8_t i=0;i<8;++i) {
    pinMode(STEP_PIN[i],OUTPUT); digitalWrite(STEP_PIN[i],LOW);
    pinMode(DIR_PIN[i],OUTPUT); digitalWrite(DIR_PIN[i],LOW);
    pinMode(LIMIT_PIN[i],INPUT_PULLUP);
  }
  Serial.begin(115200); Serial1.begin(115200); Serial2.begin(115200);
  wdt_enable(WDTO_2S);
  Serial.println(F("RoboDog MotorMap: NO AUTO MOTION. Support body and every limb."));
  Serial.println(F("600mA RMS, 1/16, gear 12. SCAN -> J1 -> ARM -> J +50"));
  Serial.println(F("! = immediate motor power off. UART 115200, Newline."));
}
void loop() {
  wdt_reset();
  if(powered && !runAllowed()) fail(F("RUN/ESTOP"));
  if((moving || armed) && !allLimitsOpen()) fail(F("limit pressed"));
  const uint32_t now=millis();
  if(armed && now-armedAt>=10000UL) { stopMotion(); Serial.println(F("ARM expired")); }
  if(powered && now-lastHost>=30000UL) fail(F("host idle 30s"));
  if(now-lastSense>=50) {
    lastSense=now;
    const uint16_t bus=voltage(A0), a=voltage(A1), b=voltage(A2);
    if(powered && (bus<24000 || bus>29500)) fail(F("VBUS low/invalid"));
    if(powered && (a<17000 || b<17000 || a>24500 || b>24500)) fail(F("motor rail / A1,A2 invalid"));
    if(a>24000) brakeA=true; else if(a<22000) brakeA=false;
    if(b>24000) brakeB=true; else if(b<22000) brakeB=false;
    digitalWrite(BRAKE_A,brakeA); digitalWrite(BRAKE_B,brakeB);
  }
  // Drain input before generating a pulse. Any new byte during a jog stops it.
  while(Serial.available()) {
    const char c=(char)Serial.read();
    if(c=='!') { powerOff(); commandLength=0; overflow=false; Serial.println(F("STOP !")); continue; }
    if(moving) { powerOff(); commandLength=0; overflow=true; Serial.println(F("STOP: input during jog")); }
    if(c=='\r') continue;
    if(c=='\n') {
      if(!overflow && commandLength) { command[commandLength]=0; handleCommand(); }
      commandLength=0; overflow=false;
    } else if(!overflow) {
      if(commandLength<sizeof(command)-1) command[commandLength++]=c;
      else { powerOff(); overflow=true; commandLength=0; }
    }
  }
  if(moving && (int32_t)(micros()-nextStep)>=0) {
    if(!runAllowed() || !allLimitsOpen()) { fail(F("interlock before STEP")); return; }
    digitalWrite(STEP_PIN[selected],HIGH); delayMicroseconds(5);
    digitalWrite(STEP_PIN[selected],LOW);
    position[selected]+=direction; budget[selected]++; remaining--;
    nextStep=micros()+5000UL; // <=200 pulses/s; never burst to catch up.
    if(!remaining) {
      stopMotion();
      driver[selected].toff(0);
      Serial.println(F("JOG DONE; disabled. ARM required again.")); printStatus();
    }
  }
}
