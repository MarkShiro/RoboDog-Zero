#pragma once
#include <stdint.h>

// Commissioning gates are set only after the physical checks in docs/commissioning.md.
constexpr bool ROBOT_CALIBRATED = false;
constexpr bool ESTOP_SIGNAL_VERIFIED = false; // separate D41 status contact
constexpr bool RAIL_SENSING_VERIFIED = false; // A1/A2, 100k/20k, measured VREF
constexpr bool DRIVER_RSENSE_VERIFIED = false; // external R110, both boards
constexpr bool ENABLE_EXPERIMENTAL_TROT = false;
// Final assembled robot: home all eight joints from their limit switches.
// For a separate supported bench-only build without switches, change 0 to 1.
#ifndef ROBODOG_STARTUP_ZERO_MODE
#define ROBODOG_STARTUP_ZERO_MODE 0
#endif
constexpr bool STARTUP_ZERO_MODE = ROBODOG_STARTUP_ZERO_MODE != 0;
constexpr bool HARDWARE_MOTOR_CUT_VERIFIED = false;
// No automatic whole-robot power cut until the Pi shutdown path is validated.
constexpr bool AUTOMATIC_POWER_RELEASE = false;

// Physical input indices 0..7 mean Mega D42..D49. Each J uses one unique input.
// Change the index if two sensors were plugged into each other's inputs.
// true: NO-to-GND, LOW when pressed; false: NC-to-GND, HIGH when pressed.
// Confirm every entry by pressing the actual joint switch with motor power OFF.
const uint8_t LIMIT_INPUT_FOR_J[8] = {0,1,2,3,4,5,6,7};
const bool LIMIT_ACTIVE_LOW_FOR_J[8] = {true,true,true,true,true,true,true,true};

inline bool validLimitInputs(const uint8_t* inputs) {
  uint8_t seen=0;
  for(uint8_t j=0;j<8;++j) {
    if(inputs[j]>=8 || (seen & (1U<<inputs[j]))) return false;
    seen |= 1U<<inputs[j];
  }
  return seen==255;
}

// One row per ELECTRICAL channel J1..J8, not anatomical order.
// leg: 0 front-left, 1 front-right, 2 rear-left, 3 rear-right.
// joint: 0 proximal, 1 distal. angleSign: electrical +STEP -> angle +/-.
// Angles in hundredths of a degree in a MEASURED joint coordinate system.
// homeAngle is the angle AT the switch; soft bounds exclude the switch.
struct JointCalibration {
  uint8_t leg, joint;
  int8_t angleSign, homeDir;
  int16_t homeAngle, minAngle, maxAngle, standAngle, sitAngle;
  uint16_t amplitude, maxHomeSteps;
};
const JointCalibration JOINTS[8] = {
  {255,255,0,0,0,0,0,0,0,0,0}, // J1 Serial1 address 0 STEP22 DIR30
  {255,255,0,0,0,0,0,0,0,0,0}, // J2 Serial1 address 1 STEP23 DIR31
  {255,255,0,0,0,0,0,0,0,0,0}, // J3 Serial1 address 2 STEP24 DIR32
  {255,255,0,0,0,0,0,0,0,0,0}, // J4 Serial1 address 3 STEP25 DIR33
  {255,255,0,0,0,0,0,0,0,0,0}, // J5 Serial2 address 0 STEP26 DIR34
  {255,255,0,0,0,0,0,0,0,0,0}, // J6 Serial2 address 1 STEP27 DIR35
  {255,255,0,0,0,0,0,0,0,0,0}, // J7 Serial2 address 2 STEP28 DIR36
  {255,255,0,0,0,0,0,0,0,0,0}  // J8 Serial2 address 3 STEP29 DIR37
};

// 200 full steps * 16 microsteps * 12 gear ratio / 36000 centidegrees.
inline int32_t angleToSteps(const JointCalibration& c, int32_t angle) {
  return (angle - c.homeAngle) * 38400L * c.angleSign / 36000L;
}
inline bool withinJoint(const JointCalibration& c, int32_t steps) {
  const int32_t a=angleToSteps(c,c.minAngle), b=angleToSteps(c,c.maxAngle);
  return steps >= (a<b?a:b) && steps <= (a>b?a:b);
}
inline bool validCalibrationRows(const JointCalibration* rows) {
  uint8_t seen=0;
  for(uint8_t i=0;i<8;++i) {
    const JointCalibration& c=rows[i];
    if(c.leg>3 || c.joint>1 || (c.angleSign!=1 && c.angleSign!=-1) ||
       (c.homeDir!=1 && c.homeDir!=-1)) return false;
    const uint8_t bit=1U<<(c.leg*2+c.joint);
    if(seen&bit) return false;
    seen |=bit;
    if(c.minAngle < -18000 || c.maxAngle > 18000 || c.minAngle>=c.maxAngle ||
       c.homeAngle < -18000 || c.homeAngle > 18000 ||
       c.standAngle<c.minAngle || c.standAngle>c.maxAngle ||
       c.sitAngle<c.minAngle || c.sitAngle>c.maxAngle || c.amplitude>500 ||
       (int32_t)c.standAngle-c.amplitude<c.minAngle ||
       (int32_t)c.standAngle+c.amplitude>c.maxAngle ||
       c.maxHomeSteps<200 || c.maxHomeSteps>12800) return false;
    // Retract must land inside the calibrated operating interval.
    if(!withinJoint(c,-c.homeDir*200L)) return false;
    // Both running bounds must be on the retract side of the home switch.
    if(angleToSteps(c,c.minAngle)*c.homeDir>=0 ||
       angleToSteps(c,c.maxAngle)*c.homeDir>=0) return false;
  }
  // A physical board must contain both joints of two legs on ONE side.
  for(uint8_t base=0;base<8;base+=4)
    for(uint8_t i=base+1;i<base+4;++i)
      if((rows[i].leg&1)!=(rows[base].leg&1)) return false;
  return seen==255 && ((rows[0].leg&1)!=(rows[4].leg&1));
}
inline bool calibrationReady() {
  if(!(ROBOT_CALIBRATED && HARDWARE_MOTOR_CUT_VERIFIED &&
       RAIL_SENSING_VERIFIED && DRIVER_RSENSE_VERIFIED)) return false;
  if(!STARTUP_ZERO_MODE) return ESTOP_SIGNAL_VERIFIED &&
    validLimitInputs(LIMIT_INPUT_FOR_J) && validCalibrationRows(JOINTS);
  uint8_t seen=0;
  for(uint8_t i=0;i<8;++i) {
    const JointCalibration& c=JOINTS[i];
    if(c.leg>3 || c.joint>1 || (c.angleSign!=1 && c.angleSign!=-1) ||
       c.homeAngle!=0 || c.minAngle< -4000 || c.maxAngle>4000 ||
       c.minAngle>=0 || c.maxAngle<=0 || c.amplitude>500 ||
       c.standAngle<c.minAngle || c.standAngle>c.maxAngle ||
       c.sitAngle<c.minAngle || c.sitAngle>c.maxAngle ||
       (int32_t)c.standAngle-c.amplitude<c.minAngle ||
       (int32_t)c.standAngle+c.amplitude>c.maxAngle) return false;
    const uint8_t bit=1U<<(c.leg*2+c.joint);
    if(seen&bit) return false;
    seen|=bit;
  }
  for(uint8_t base=0;base<8;base+=4)
    for(uint8_t i=base+1;i<base+4;++i)
      if((JOINTS[i].leg&1)!=(JOINTS[base].leg&1)) return false;
  return seen==255 && ((JOINTS[0].leg&1)!=(JOINTS[4].leg&1));
}
