// =====================================================================
//  link.h — обмен с Orange Pi по Serial3 (выводы 14 и 15).
//
//  C/T are remote-compatible. Pi must implement freshness and source ownership.
//  ARM is a separate A,<latest neutral C sequence> frame; no automatic rearm.
//
//  Приём:   C,<seq>,<кнопки hex>,<мВ пульта>,<скорость>,<режим>*<CRC>
//           A,<seq>*<CRC>          — explicit HOME + calibrated STAND request
//           K is ignored; acceptance of poweroff is not proof of OS halt.
//  Отправка: T,<seq>,<состояние>,<мВ шины>,<флаги hex>*<CRC>
//           X*<CRC>                — команда Orange Pi выключаться
// =====================================================================
#pragma once
#include "config.h"

// Биты в поле «кнопки» — те же номера, что в прошивке пульта
enum : uint16_t {
  CMD_FWD   = 1 << 0,
  CMD_BACK  = 1 << 1,
  CMD_LEFT  = 1 << 2,
  CMD_RIGHT = 1 << 3,
  CMD_STAND = 1 << 4,
  CMD_MODE  = 1 << 5,
  CMD_SPDDN = 1 << 6,
  CMD_SPDUP = 1 << 7,
  CMD_MENU  = 1 << 8,
  CMD_STOP  = 1 << 9
};

// Биты поля «флаги» в телеметрии
enum : uint16_t {
  FLG_DRIVERS_OK = 1 << 0,
  FLG_IMU_OK     = 1 << 1,
  FLG_HOMED      = 1 << 2,
  FLG_MOT_POWER  = 1 << 3,
  FLG_BRAKE      = 1 << 4,
  FLG_LOWBAT     = 1 << 5,
  FLG_NO_CTRL    = 1 << 6,
  FLG_SENSE_BAD  = 1 << 7,
  FLG_RELATIVE_ZERO = 1 << 8
};

struct Control {
  uint16_t buttons = 0;
  uint16_t remoteMv = 0;
  uint8_t  speed = 0;
  uint8_t  mode = 0;
};

class Link {
public:
  void begin();
  void poll();

  bool fresh() const;                       // команды приходят
  const Control& ctrl() const { return _c; }
  bool piAcked() const { return _piAck; }
  bool takeArmRequest() { bool a=_arm; _arm=false; return a; }
  bool takeStopRequest() { bool s=_stop; _stop=false; return s; }
  void cancelArm() { _arm=false; }
  uint16_t remainingMs() const {
    const uint32_t age=millis()-_lastRx;
    return _ever && age<CTRL_TIMEOUT_MS ? CTRL_TIMEOUT_MS-age : 0;
  }

  void sendTelemetry(uint8_t state, uint16_t vbusMv, uint16_t flags);
  void sendImu(int16_t rollDeg, int16_t pitchDeg, int16_t tempC, uint16_t flags);
  void sendShutdown();                      // просим Orange Pi выключиться

  uint16_t rxCount() const  { return _rx; }
  uint16_t crcErrors() const { return _crcErr; }

private:
  void handleLine();
  static uint8_t crc8(const char* s, uint8_t len);

  Control  _c;
  char     _buf[56];
  uint8_t  _len = 0;
  uint16_t _seq = 0, _rx = 0, _crcErr = 0;
  uint32_t _lastRx = 0, _nextTx = 0;
  uint32_t _nextImu = 0;
  bool     _ever = false, _piAck = false;
  bool _overflow=false, _arm=false, _stop=false, _armSeen=false;
  uint16_t _controlSeq=0, _armSeq=0;
};
