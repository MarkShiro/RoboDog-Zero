#include "power.h"

void Power::begin() {
  // Удержание питания поднимаем раньше всего остального: если контроллер
  // задержится здесь, кнопку пуска придётся держать дольше.
  pinMode(PIN_KEEP_ALIVE, OUTPUT);
  digitalWrite(PIN_KEEP_ALIVE, HIGH);

  pinMode(PIN_MOT_PWR, OUTPUT);
  digitalWrite(PIN_MOT_PWR, LOW);       // моторы обесточены по умолчанию

  pinMode(PIN_PWR_REQ, INPUT_PULLUP);
  pinMode(PIN_ESTOP,   INPUT_PULLUP);

  pinMode(PIN_FAN,     OUTPUT);  analogWrite(PIN_FAN, 0);
  pinMode(PIN_BRAKE_F, OUTPUT);  digitalWrite(PIN_BRAKE_F, LOW);
  pinMode(PIN_BRAKE_R, OUTPUT);  digitalWrite(PIN_BRAKE_R, LOW);

  analogRead(PIN_VBUS);                 // прогреть АЦП
  delay(2);
  _vbus = readMv(PIN_VBUS);
  _v20f = readMv(PIN_V20F);
  _v20r = readMv(PIN_V20R);
}

uint16_t Power::readMv(uint8_t pin) {
  // Четыре выборки подряд: усреднение почти бесплатное, а наводки от
  // моторных жгутов на длинном проводе делителя вполне реальные.
  analogRead(pin); // discard first conversion after ADC multiplexer change
  uint16_t acc = 0;
  for (uint8_t i = 0; i < 4; i++) acc += analogRead(pin);
  const uint32_t code = acc / 4;
  return (uint16_t)(code * VREF_MV * DIV_MUL / 1023UL);
}

void Power::update() {
  const uint32_t now = millis();
  if ((int32_t)(now - _next) < 0) return;
  _next = now + SENSE_PERIOD_MS;

  // Сглаживание: три четверти старого значения плюс четверть нового.
  _vbus = (uint16_t)((3UL * _vbus + readMv(PIN_VBUS)) / 4);
  _v20f = (uint16_t)((3UL * _v20f + readMv(PIN_V20F)) / 4);
  _v20r = (uint16_t)((3UL * _v20r + readMv(PIN_V20R)) / 4);

  _runReq = (digitalRead(PIN_PWR_REQ) == LOW);
  _estop  = ESTOP_SIGNAL_VERIFIED && (digitalRead(PIN_ESTOP)==LOW);

  // Тормозные ключи с гистерезисом: при резкой остановке ноги моторы
  // возвращают энергию в шину, и напряжение на ней подскакивает.
  // Ключ сжигает избыток на цементном резисторе, не давая дойти до
  // предельных для TMC2209 29 вольт.
  if (!_brakeF && _v20f > V20_BRAKE_MV)  _brakeF = true;
  if (_brakeF  && _v20f < V20_BRAKE_OFF) _brakeF = false;
  if (!_brakeR && _v20r > V20_BRAKE_MV)  _brakeR = true;
  if (_brakeR  && _v20r < V20_BRAKE_OFF) _brakeR = false;
  if(!RAIL_SENSING_VERIFIED) _brakeF=_brakeR=false;
  digitalWrite(PIN_BRAKE_F, _brakeF ? HIGH : LOW);
  digitalWrite(PIN_BRAKE_R, _brakeR ? HIGH : LOW);
}

void Power::motorsPower(bool on) {
  _motPwr = on;
  digitalWrite(PIN_MOT_PWR, on ? HIGH : LOW);
}

void Power::fan(uint8_t duty) { analogWrite(PIN_FAN, duty); }

void Power::releasePower() {
  // После этой строки контроллер обесточится вместе со всем роботом.
  digitalWrite(PIN_MOT_PWR, LOW);
  digitalWrite(PIN_KEEP_ALIVE, LOW);
}
