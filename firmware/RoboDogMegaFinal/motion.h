// =====================================================================
//  motion.h — поиск нулей суставов, позы и ограниченная диагональная походка.
//
//  Слой между «крутить мотор» и «идти вперёд». Каждый сустав имеет
//  целевую позицию в шагах, а сюда сводится вся логика, которая эти
//  позиции назначает.
//
//  Амплитуды, фазы и направления требуют измерений на собранном роботе.
//  Походка включается отдельно после проверки калибровки на опоре.
// =====================================================================
#pragma once
#include "config.h"

enum Pose : uint8_t { POSE_ZERO = 0, POSE_STAND, POSE_SIT };

class Motion {
public:
  void begin();

  // --- поиск нулей
  void startHoming();
  void startRelativeZero();
  bool homing() const { return _hom != HOM_DONE && _hom != HOM_IDLE; }
  bool homed() const  { return _homed; }
  uint8_t homingJoint() const { return _j; }

  // --- обычная работа
  void setPose(Pose p);
  void walk(int8_t fwd, int8_t turn, uint8_t speed);   // -1..1, -1..1, 0..5
  void halt();                 // immediate cancel; invalidate homing and positions
  void hold();
  bool fault() const { return _fault; }
  bool limitPressed(uint8_t j) const;

  void update();          // звать в каждом проходе loop()
  bool moving() const;

private:
  enum HomState : uint8_t {
    HOM_IDLE = 0, HOM_BACKOFF, HOM_SEEK, HOM_RETRACT, HOM_NEXT, HOM_DONE
  };

  void servo(uint8_t j, int32_t target, uint16_t maxSps);

  HomState _hom = HOM_IDLE;
  uint8_t  _j = 0;
  uint32_t _t0 = 0;
  bool     _homed = false;
  bool _active=false, _fault=false;
  int32_t _origin=0;

  int32_t  _target[MOTOR_COUNT];
  bool     _walking = false;
  int8_t   _fwd = 0, _turn = 0;
  uint8_t  _speed = 2;
  uint32_t _phaseMs = 0;
  uint32_t _lastPhase = 0;
};
