// =====================================================================
//  power.h — самоудержание питания, измерение напряжений,
//  тормозные ключи и вентиляторы.
//
//  Главное свойство схемы: питание робота держит сам контроллер через
//  транзистор Q7. Пока вывод KEEP_ALIVE в единице — робот жив. Опустить
//  его означает выключить всё, включая сам контроллер, поэтому делать
//  это можно только в самом конце последовательности выключения.
// =====================================================================
#pragma once
#include "config.h"

class Power {
public:
  void begin();          // ПЕРВОЕ, что должно выполниться в setup()
  void update();         // измерения, тормозные ключи, вентилятор

  // --- измерения, милливольты
  uint16_t vbus() const  { return _vbus; }
  uint16_t v20f() const  { return _v20f; }
  uint16_t v20r() const  { return _v20r; }
  bool     senseSane() const { return _vbus > VBUS_SANE_MV && _vbus<=29500; }
  bool railsSane() const { return RAIL_SENSING_VERIFIED && _v20f>=17000 && _v20f<=24500 && _v20r>=17000 && _v20r<=24500; }
  bool braking() const { return _brakeF || _brakeR; }

  // --- органы управления
  bool runRequested() const  { return _runReq; }   // тумблер SW1 замкнут
  bool estopPressed() const  { return _estop; }    // грибок нажат

  void motorsPower(bool on);
  bool motorsPowered() const { return _motPwr; }

  void fan(uint8_t duty);            // 0..255
  void releasePower();               // снять KEEP_ALIVE: точка невозврата

private:
  uint16_t readMv(uint8_t pin);

  uint16_t _vbus = 0, _v20f = 0, _v20r = 0;
  bool _runReq = false, _estop = false, _motPwr = false;
  bool _brakeF = false, _brakeR = false;
  uint32_t _next = 0;
};
