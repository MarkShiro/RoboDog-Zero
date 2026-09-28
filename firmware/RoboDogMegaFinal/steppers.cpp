#include <TMCStepper.h>
#include "steppers.h"

namespace {

// --- состояние генератора, к которому обращается прерывание
volatile uint16_t g_accum[MOTOR_COUNT];
volatile uint16_t g_inc[MOTOR_COUNT];      // приращение накопителя за тик
volatile int32_t  g_pos[MOTOR_COUNT];
volatile uint8_t  g_dirBits;               // готовое значение для PORTC
volatile int8_t   g_sign[MOTOR_COUNT];

// --- состояние, которое живёт только в основном цикле
int16_t  g_target[MOTOR_COUNT];            // куда хотим, шагов в секунду
int16_t  g_cur[MOTOR_COUNT];               // где сейчас
uint32_t g_lastTick;
volatile bool g_enabled = false;
volatile bool g_timingFault = false;
volatile uint16_t g_lease = 0;
volatile bool g_bounds=false;
int32_t g_min[MOTOR_COUNT],g_max[MOTOR_COUNT];
uint8_t  g_online  = 0;

// Драйверы. Адрес внутри своей шины — 0..3, задаётся выводами MS1 и MS2.
TMC2209Stepper g_drv[MOTOR_COUNT] = {
  TMC2209Stepper(&Serial1, TMC_RSENSE, 0b00),
  TMC2209Stepper(&Serial1, TMC_RSENSE, 0b01),
  TMC2209Stepper(&Serial1, TMC_RSENSE, 0b10),
  TMC2209Stepper(&Serial1, TMC_RSENSE, 0b11),
  TMC2209Stepper(&Serial2, TMC_RSENSE, 0b00),
  TMC2209Stepper(&Serial2, TMC_RSENSE, 0b01),
  TMC2209Stepper(&Serial2, TMC_RSENSE, 0b10),
  TMC2209Stepper(&Serial2, TMC_RSENSE, 0b11)
};

// Скорость в шагах в секунду -> приращение накопителя.
// 65536 приращений накопителя = один шаг.
inline uint16_t spsToInc(uint16_t sps) {
  if (sps > SPEED_MAX_SPS) sps = SPEED_MAX_SPS;
  return (uint16_t)(((uint32_t)sps * 65536UL) / STEP_ISR_HZ);
}

}  // namespace

// Прерывание таймера. Частота задаётся STEP_ISR_HZ,
// поэтому никаких вызовов библиотек и никакой арифметики сверх необходимой.
ISR(TIMER1_COMPA_vect) {
  PORTA = 0;                       // снять импульсы, поставленные прошлым тиком
  if (!g_enabled) return;
  if (!g_lease || --g_lease==0) {
    g_enabled=false; g_timingFault=true;
    PORTD |= (1<<7); // Mega D38 / PD7 = EN high, immediate hardware disable
    return;
  }
  // Allow a full timer interval for DIR setup before any STEP edge.
  if(PORTC != g_dirBits) { PORTC=g_dirBits; return; }
  PORTC = g_dirBits;               // направления действуют до фронта STEP

  uint8_t bits = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    const uint16_t inc = g_inc[i];
    if (!inc) continue;
    const uint16_t a = (uint16_t)(g_accum[i] + inc);
    if (a < g_accum[i]) {          // накопитель переполнился — время шага
      const int32_t next=g_pos[i]+g_sign[i];
      if(g_bounds && (next<g_min[i] || next>g_max[i])) {
        g_enabled=false; g_timingFault=true; PORTD|=(1<<7); return;
      }
      bits |= (uint8_t)(1 << i);
    }
    g_accum[i] = a;
  }
  for(uint8_t i=0;i<MOTOR_COUNT;++i) if(bits & (1<<i)) g_pos[i]+=g_sign[i];
  PORTA = bits;                    // фронт сразу на всех нужных моторах
}

namespace Steppers {

void begin() {
  DDRA  = 0xFF;  PORTA = 0x00;     // D22..D29 — выходы STEP
  DDRC  = 0xFF;  PORTC = 0x00;     // D30..D37 — выходы DIR
  pinMode(PIN_DRV_EN, OUTPUT);
  digitalWrite(PIN_DRV_EN, HIGH);  // драйверы запрещены до явной команды

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    g_accum[i] = 0; g_inc[i] = 0; g_pos[i] = 0;
    g_sign[i] = 1;  g_target[i] = 0; g_cur[i] = 0;
  }
  g_dirBits = 0;

  // Таймер 1 в режиме CTC, предделитель 8.
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11);
  OCR1A  = (uint16_t)(F_CPU / 8UL / STEP_ISR_HZ) - 1;
  TCNT1  = 0;
  TIMSK1 = (1 << OCIE1A);
  interrupts();

  g_lastTick = millis();
}

bool setupDrivers(bool (*service)()) {
  Serial1.begin(TMC_BAUD);
  Serial2.begin(TMC_BAUD);
  delay(20);

  g_online = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if(!service()) return false;
    g_drv[i].begin();
    g_drv[i].pdn_disable(true);
    g_drv[i].I_scale_analog(false);
    g_drv[i].internal_Rsense(false);
    g_drv[i].mstep_reg_select(true);
    g_drv[i].toff(4);                       // включить выходной каскад
    g_drv[i].blank_time(24);
    g_drv[i].rms_current(TMC_RUN_MA, TMC_HOLD_MULT);
    g_drv[i].microsteps(TMC_MICROSTEPS);
    g_drv[i].en_spreadCycle(true);
    g_drv[i].pwm_autoscale(true);
    g_drv[i].iholddelay(8);                 // плавный переход к току покоя
    g_drv[i].TPOWERDOWN(20);
    const uint8_t before=g_drv[i].IFCNT();
    g_drv[i].toff(4);
    const uint8_t after=g_drv[i].IFCNT();
    if ((uint8_t)(after-before)==1 && !g_drv[i].I_scale_analog() &&
        !g_drv[i].internal_Rsense() && g_drv[i].mstep_reg_select() &&
        g_drv[i].microsteps()==TMC_MICROSTEPS && checkDriver(i)) g_online++;
    else return false;
  }
  return service() && g_online == MOTOR_COUNT;
}

bool checkDriver(uint8_t i) {
  if(i>=MOTOR_COUNT) return false;
  uint32_t status=g_drv[i].DRV_STATUS();
  return (uint8_t)(g_drv[i].IOIN()>>24)==0x21 && status!=0xFFFFFFFFUL &&
    !(status & 0x3FUL) && g_drv[i].toff()==4 &&
    !g_drv[i].I_scale_analog() && g_drv[i].mstep_reg_select() &&
    g_drv[i].microsteps()==TMC_MICROSTEPS;
}
bool timingFault() { return g_timingFault; }
void setBounds(bool on) {
  int32_t lo[MOTOR_COUNT],hi[MOTOR_COUNT];
  for(uint8_t i=0;i<MOTOR_COUNT;++i) {
    int32_t a=angleToSteps(JOINTS[i],JOINTS[i].minAngle);
    int32_t b=angleToSteps(JOINTS[i],JOINTS[i].maxAngle);
    lo[i]=a<b?a:b; hi[i]=a>b?a:b;
  }
  noInterrupts();
  for(uint8_t i=0;i<MOTOR_COUNT;++i) { g_min[i]=lo[i]; g_max[i]=hi[i]; }
  g_bounds=on; interrupts();
}
void renewLease(uint16_t controlRemainingMs) {
  if(controlRemainingMs>250) controlRemainingMs=250;
  noInterrupts();
  if(g_enabled && !g_timingFault) g_lease=controlRemainingMs*(STEP_ISR_HZ/1000);
  interrupts();
}

uint8_t driversOnline() { return g_online; }

void enable(bool on) {
  stopAll();
  noInterrupts();
  if(on) { g_timingFault=false; g_lease=STEP_ISR_HZ/4; }
  g_enabled = on;
  digitalWrite(PIN_DRV_EN, on ? LOW : HIGH);   // активный низкий
  interrupts();
}

bool enabled() { return g_enabled; }

void setSpeed(uint8_t m, int16_t sps) {
  if (m >= MOTOR_COUNT || !g_enabled) return;
  if (sps >  (int16_t)SPEED_MAX_SPS) sps =  (int16_t)SPEED_MAX_SPS;
  if (sps < -(int16_t)SPEED_MAX_SPS) sps = -(int16_t)SPEED_MAX_SPS;
  g_target[m] = sps;
}

void setAllSpeeds(int16_t sps) {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) setSpeed(i, sps);
}

void stopAll() {
  noInterrupts();
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    g_inc[i] = 0; g_target[i] = 0; g_cur[i] = 0; g_accum[i] = 0;
  }
  PORTA=0;
  interrupts();
}

void tick() {
  if(!g_enabled) { stopAll(); g_lastTick=millis(); return; }
  const uint32_t now = millis();
  uint16_t dt = (uint16_t)(now - g_lastTick);
  if (dt == 0) return;
  if (dt > 100) dt = 100;                   // после задержки не прыгаем скачком
  g_lastTick = now;

  // Максимальное изменение скорости за этот интервал
  int32_t maxStep = ((int32_t)ACCEL_SPS2 * dt) / 1000;
  if(maxStep<1) maxStep=1;
  uint8_t dirBits = 0;
  int8_t signs[MOTOR_COUNT]; uint16_t increments[MOTOR_COUNT];

  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    // Направление меняем только через полную остановку: смена DIR на ходу
    // даёт пропуск шага и рывок в суставе. Поэтому если знак цели
    // противоположен текущему, сначала тормозим до нуля.
    int16_t aim = g_target[i];
    if ((aim > 0 && g_cur[i] < 0) || (aim < 0 && g_cur[i] > 0)) aim = 0;

    int32_t d = (int32_t)aim - g_cur[i];
    if (d >  maxStep) d =  maxStep;
    if (d < -maxStep) d = -maxStep;
    g_cur[i] = (int16_t)(g_cur[i] + d);

    // Знак всегда соответствует текущей скорости, а на нуле — будущей.
    if (g_cur[i] > 0)      signs[i] = 1;
    else if (g_cur[i] < 0) signs[i] = -1;
    else                   signs[i] = (g_target[i] >= 0) ? 1 : -1;

    const uint16_t mag = (uint16_t)((g_cur[i] >= 0) ? g_cur[i] : -g_cur[i]);

    // D30 это PC7, D31 это PC6 и так далее — порядок обратный номеру мотора.
    if (signs[i] > 0) dirBits |= (uint8_t)(1 << (7 - i));

    const uint16_t inc = spsToInc(mag);
    increments[i]=inc;
  }

  noInterrupts();
  for(uint8_t i=0;i<MOTOR_COUNT;++i) { g_inc[i]=increments[i]; g_sign[i]=signs[i]; }
  g_dirBits = dirBits;
  interrupts();
}

int32_t position(uint8_t m) {
  if (m >= MOTOR_COUNT) return 0;
  noInterrupts();
  const int32_t p = g_pos[m];
  interrupts();
  return p;
}

void zero(uint8_t m) {
  if (m >= MOTOR_COUNT) return;
  noInterrupts();
  g_pos[m] = 0;
  interrupts();
}

int16_t speed(uint8_t m) { return (m < MOTOR_COUNT) ? g_cur[m] : 0; }

void setHoldMultiplier(float k) {
  if (k < 0.05f) k = 0.05f;
  if (k > 1.0f)  k = 1.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) g_drv[i].rms_current(TMC_RUN_MA, k);
}

}  // namespace Steppers
