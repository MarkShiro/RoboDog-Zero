#include <Wire.h>
#include "imu.h"

namespace {
constexpr uint8_t ADDR        = 0x68;   // вывод ADD датчика на землю
constexpr uint8_t REG_PWR     = 0x6B;
constexpr uint8_t REG_WHOAMI  = 0x75;
constexpr uint8_t REG_ACCEL   = 0x3B;

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
}  // namespace

bool Imu::begin() {
  Wire.begin();
  Wire.setWireTimeout(25000,true); // bound a stuck I2C bus
  Wire.setClock(400000);

  Wire.beginTransmission(ADDR);
  Wire.write(REG_WHOAMI);
  if (Wire.endTransmission(false) != 0) { _ok = false; return false; }
  if (Wire.requestFrom((uint8_t)ADDR, (uint8_t)1) != 1) { _ok = false; return false; }
  const uint8_t who = Wire.read();
  // Родной MPU-6050 отвечает 0x68, распространённые клоны — 0x70 или 0x72.
  if (who != 0x68) { _ok = false; return false; } // do not assume other IDs are MPU6050

  _ok = writeReg(REG_PWR, 0x00);        // снять режим сна
  return _ok;
}

void Imu::update() {
  if (!_ok) return;
  const uint32_t now = millis();
  if ((int32_t)(now - _next) < 0) return;
  _next = now + 10;

  Wire.beginTransmission(ADDR);
  Wire.write(REG_ACCEL);
  if (Wire.endTransmission(false) != 0) { _ok = false; return; }
  if (Wire.requestFrom((uint8_t)ADDR, (uint8_t)8) != 8) { _ok = false; return; }

  const int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  const int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
  const int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
  const int16_t traw = (int16_t)((Wire.read() << 8) | Wire.read());

  _temp = (int16_t)(traw / 340 + 36);

  // Углы по вектору силы тяжести. Для стояния и медленной походки этого
  // достаточно; на рывках акселерометр врёт, и тогда понадобится
  // объединение с гироскопом — но это уже слой стабилизации, не датчика.
  const float fax = ax, fay = ay, faz = az;
  _roll  = (int16_t)(atan2(fay, faz) * 57.2958f);
  _pitch = (int16_t)(atan2(-fax, sqrt(fay * fay + faz * faz)) * 57.2958f);
}
