#include "motion.h"
#include "steppers.h"
#include <math.h>
namespace {
constexpr int16_t BACKOFF=200;
constexpr uint16_t PERIOD[6]={6000,5500,5000,4500,4000,3500};
const uint8_t LEG_PHASE[4]={0,128,128,0};
int32_t magnitude(int32_t x) { return x<0?-x:x; }
}
void Motion::begin() {
  for(uint8_t i=0;i<MOTOR_COUNT;++i) { pinMode(PIN_LIMIT[i],INPUT_PULLUP); _target[i]=0; }
}
bool Motion::limitPressed(uint8_t j) const { return digitalRead(PIN_LIMIT[j])==LOW; }
void Motion::servo(uint8_t j,int32_t target,uint16_t maxSps) {
  int32_t err=target-Steppers::position(j);
  if(magnitude(err)<=2) { Steppers::setSpeed(j,0); return; }
  int32_t speed=magnitude(err)*4;
  int32_t braking=(int32_t)sqrt(2.0f*ACCEL_SPS2*magnitude(err));
  if(speed>braking) speed=braking;
  if(speed>maxSps) speed=maxSps;
  Steppers::setSpeed(j,(int16_t)(err>0?speed:-speed));
}
void Motion::startHoming() {
  halt();
  if(!calibrationReady() || !Steppers::enabled()) { _fault=true; return; }
  Steppers::setBounds(false);
  _fault=false; _active=true; _j=0; _hom=HOM_BACKOFF;
  _t0=millis(); _origin=Steppers::position(0);
}
void Motion::startRelativeZero() {
  halt();
  if(!calibrationReady() || !Steppers::enabled() || !STARTUP_ZERO_MODE) {
    _fault=true; return;
  }
  for(uint8_t i=0;i<MOTOR_COUNT;++i) {
    Steppers::zero(i); _target[i]=0;
  }
  _fault=false; _homed=true; _active=true; _hom=HOM_DONE;
  Steppers::setBounds(true);
}
void Motion::halt() {
  _walking=false; _active=false; _homed=false; _hom=HOM_IDLE;
  _lastPhase=0; _phaseMs=0;
  Steppers::stopAll();
}
void Motion::hold() {
  if(!_homed || !_active) return;
  _walking=false; _lastPhase=0; _phaseMs=0;
  Steppers::stopAll();
  for(uint8_t i=0;i<MOTOR_COUNT;++i) _target[i]=Steppers::position(i);
}
void Motion::setPose(Pose pose) {
  if(!_homed || !_active || (pose!=POSE_STAND && pose!=POSE_SIT)) return;
  _walking=false;
  for(uint8_t i=0;i<MOTOR_COUNT;++i)
    _target[i]=angleToSteps(JOINTS[i],pose==POSE_SIT?JOINTS[i].sitAngle:JOINTS[i].standAngle);
}
void Motion::walk(int8_t fwd,int8_t turn,uint8_t speed) {
  if(!_homed || !_active) return;
  if(!fwd && !turn) { if(_walking) hold(); return; }
  if(!ENABLE_EXPERIMENTAL_TROT) { hold(); return; }
  _fwd=fwd; _turn=turn; _speed=speed>5?5:speed;
  if(!_walking) { _lastPhase=millis(); _phaseMs=0; }
  _walking=true;
}
bool Motion::moving() const {
  for(uint8_t i=0;i<MOTOR_COUNT;++i) if(Steppers::speed(i)) return true;
  return false;
}
void Motion::update() {
  if(!_active || !Steppers::enabled()) return;
  const uint32_t now=millis();
  if(homing()) {
    const JointCalibration& c=JOINTS[_j];
    // Bound each phase by TIME and commanded travel, including stuck switches.
    if(now-_t0>30000UL || magnitude(Steppers::position(_j)-_origin)>c.maxHomeSteps) {
      halt(); _fault=true; return;
    }
    for(uint8_t i=0;i<_j;++i) if(limitPressed(i)) { halt(); _fault=true; return; }
    switch(_hom) {
      case HOM_BACKOFF:
        if(limitPressed(_j)) Steppers::setSpeed(_j,-c.homeDir*(int16_t)HOMING_SPS);
        else {
          Steppers::stopAll(); _hom=HOM_SEEK; _t0=now; _origin=Steppers::position(_j);
        }
        break;
      case HOM_SEEK:
        if(limitPressed(_j)) {
          Steppers::stopAll(); Steppers::zero(_j);
          _hom=HOM_RETRACT; _t0=now; _origin=0;
        } else Steppers::setSpeed(_j,c.homeDir*(int16_t)HOMING_SPS);
        break;
      case HOM_RETRACT: {
        const int32_t target=-c.homeDir*(int32_t)BACKOFF;
        servo(_j,target,HOMING_SPS);
        if(magnitude(target-Steppers::position(_j))<=2 && !Steppers::speed(_j) && now-_t0>300) {
          if(limitPressed(_j)) { halt(); _fault=true; return; }
          Steppers::stopAll(); _target[_j]=target; _hom=HOM_NEXT;
        }
        break;
      }
      case HOM_NEXT:
        if(++_j==MOTOR_COUNT) {
          _hom=HOM_DONE; _homed=true;
          Steppers::setBounds(true);
          // Explicit ARM HOME includes transition into calibrated stand pose.
          setPose(POSE_STAND);
        } else { _hom=HOM_BACKOFF; _t0=now; _origin=Steppers::position(_j); }
        break;
      default: break;
    }
    return;
  }
  if(!_homed) return;
  for(uint8_t i=0;i<MOTOR_COUNT;++i) {
    if(limitPressed(i) || !withinJoint(JOINTS[i],Steppers::position(i))) { halt(); _fault=true; return; }
  }
  if(_walking) {
    const uint16_t period=PERIOD[_speed];
    _phaseMs=(_phaseMs+(now-_lastPhase))%period; _lastPhase=now;
    const uint8_t base=(uint8_t)(_phaseMs*256UL/period);
    for(uint8_t i=0;i<MOTOR_COUNT;++i) {
      const JointCalibration& c=JOINTS[i];
      const uint8_t phase=(uint8_t)(base+LEG_PHASE[c.leg]+(c.joint?64:0));
      // Differential sides, bounded +/-100%, including in-place turn.
      int16_t drive=100*_fwd+((c.leg&1)?-60:60)*_turn;
      if(drive>100) drive=100; if(drive<-100) drive=-100;
      const int32_t swing=(int32_t)(sin(phase*6.2831853f/256.0f)*c.amplitude*drive/100.0f);
      _target[i]=angleToSteps(c,(int32_t)c.standAngle+swing);
    }
  }
  for(uint8_t i=0;i<MOTOR_COUNT;++i) {
    if(!withinJoint(JOINTS[i],_target[i])) { halt(); _fault=true; return; }
    servo(i,_target[i],SPEED_MAX_SPS);
  }
}
