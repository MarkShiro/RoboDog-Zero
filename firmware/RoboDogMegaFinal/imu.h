// =====================================================================
//  imu.h — MPU-6050 напрямую через Wire, без сторонней библиотеки.
//  Нужно ровно три вещи: углы крена и тангажа и признак «датчик отвечает».
// =====================================================================
#pragma once
#include "config.h"

class Imu {
public:
  bool begin();                 // true, если датчик ответил
  void update();                // читает регистры не чаще 100 раз в секунду

  bool ok() const     { return _ok; }
  int16_t rollDeg() const  { return _roll; }    // крен, градусы
  int16_t pitchDeg() const { return _pitch; }   // тангаж, градусы
  int16_t tempC() const    { return _temp; }

private:
  bool     _ok = false;
  int16_t  _roll = 0, _pitch = 0, _temp = 0;
  uint32_t _next = 0;
};
