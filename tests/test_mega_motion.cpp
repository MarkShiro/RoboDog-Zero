#include <assert.h>
#include <stdio.h>
#include "mega_host.h"
#define ROBODOG_STARTUP_ZERO_MODE 1
// Observe the real state machine with simulated motor positions and switches.
#define private public
#include "../firmware/RoboDogMegaFinal/motion.h"
#undef private
#include "../firmware/RoboDogMegaFinal/steppers.h"
JointCalibration fixture[8]={
 {0,0,1,-1,-1000,-900,1000,0,500,300,3000},
 {0,1,1,-1,-1000,-900,1000,0,500,300,3000},
 {2,0,1,-1,-1000,-900,1000,0,500,300,3000},
 {2,1,1,-1,-1000,-900,1000,0,500,300,3000},
 {1,0,1,-1,-1000,-900,1000,0,500,300,3000},
 {1,1,1,-1,-1000,-900,1000,0,500,300,3000},
 {3,0,1,-1,-1000,-900,1000,0,500,300,3000},
 {3,1,1,-1,-1000,-900,1000,0,500,300,3000}
};
bool testCalibrationReady() { return validCalibrationRows(fixture); }
uint8_t fixtureLimitInput[8]={0,1,2,3,4,5,6,7};
bool fixtureLimitActiveLow[8]={true,true,true,true,true,true,true,true};
// Inject measured test data, leaving the shipped firmware's lock unchanged.
#define JOINTS fixture
#define calibrationReady testCalibrationReady
#define ENABLE_EXPERIMENTAL_TROT true
#define LIMIT_INPUT_FOR_J fixtureLimitInput
#define LIMIT_ACTIVE_LOW_FOR_J fixtureLimitActiveLow
#include "../firmware/RoboDogMegaFinal/motion.cpp"
#undef LIMIT_ACTIVE_LOW_FOR_J
#undef LIMIT_INPUT_FOR_J
#undef ENABLE_EXPERIMENTAL_TROT
#undef calibrationReady
#undef JOINTS
int32_t positions[8]={}; int16_t speeds[8]={}; bool enabledFlag=true;
namespace Steppers {
 bool enabled() { return enabledFlag; }
 void stopAll() { for(auto& s:speeds) s=0; }
 void setSpeed(uint8_t i,int16_t s) { speeds[i]=s; }
 int16_t speed(uint8_t i) { return speeds[i]; }
 int32_t position(uint8_t i) { return positions[i]; }
 void zero(uint8_t i) { positions[i]=0; }
 void setBounds(bool) {}
}
int main() {
 assert(validCalibrationRows(fixture));
 fixture[1].joint=0; assert(!validCalibrationRows(fixture)); fixture[1].joint=1;
 fixture[0].angleSign=0; assert(!validCalibrationRows(fixture)); fixture[0].angleSign=1;
 fixture[0].amplitude=501; assert(!validCalibrationRows(fixture)); fixture[0].amplitude=300;
 assert(validLimitInputs(fixtureLimitInput));
 fixtureLimitInput[0]=1; assert(!validLimitInputs(fixtureLimitInput));
 fixtureLimitInput[0]=8; assert(!validLimitInputs(fixtureLimitInput));
 fixtureLimitInput[0]=0;
 Motion m; m.begin(); m.startHoming(); assert(m.homing());
 pins[42]=HIGH; pins[43]=LOW; fixtureLimitInput[0]=1; fixtureLimitInput[1]=0;
 assert(m.limitPressed(0) && !m.limitPressed(1));
 pins[43]=HIGH; fixtureLimitActiveLow[0]=false;
 assert(m.limitPressed(0));
 fixtureLimitActiveLow[0]=true;
 fixtureLimitInput[0]=0; fixtureLimitInput[1]=1;
 m.update(); m.update(); assert(speeds[0]<0);
 m.halt(); m.update(); assert(!m.homing() && !m.homed() && speeds[0]==0);
 m.startHoming(); pins[42]=LOW; m.update(); assert(speeds[0]>0);
 clockUs+=31000000; m.update(); assert(m.fault() && !m.homing());
 pins[42]=HIGH; m.startHoming(); positions[0]=4000; m.update(); assert(m.fault());
 positions[0]=0; m.startHoming();
 for(int j=0;j<8;++j) {
   pins[42+j]=HIGH; m.update(); // BACKOFF -> SEEK
   pins[42+j]=LOW; m.update(); // SEEK -> RETRACT and zero
   pins[42+j]=HIGH; positions[j]=200; speeds[j]=0; clockUs+=400000;
   m.update(); m.update();
 }
 assert(m.homed() && !m.homing() && !m.fault());
 for(auto& p:positions) p=1066;
 m.walk(0,1,0); m.update(); bool turnMoves=false;
 for(int i=0;i<8;++i) if(m._target[i]!=1066) turnMoves=true;
 assert(turnMoves); // original zero-forward turn bug must not recur
 m.walk(0,0,0); assert(!m._walking);
 for(int i=0;i<8;++i) assert(m._target[i]==positions[i]);
 m.setPose(POSE_SIT); int32_t sit=m._target[0];
 m.walk(0,0,0); assert(m._target[0]==sit); // 800ms remote SIT remains latched
 pins[42]=LOW; m.update(); assert(m.fault() && !m.homed());
 m.update(); for(auto s:speeds) assert(s==0);
 pins[42]=HIGH;
 for(auto& p:positions) p=123;
 m.startRelativeZero();
 assert(m.homed() && !m.fault() && !m.homing());
 for(auto p:positions) assert(p==0);
 m.halt(); assert(!m.homed());
 puts("PASS Mega motion: calibration validation, homing bounds, retract, turn, hold, sit, limit fault, relative zero");
}
