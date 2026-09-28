// =====================================================================
//  config.h — вся привязка к железу в одном месте.
//  Соответствует документу «Электросхема робота-собаки».
// =====================================================================
#pragma once
#include <Arduino.h>
#include "calibration.h"

// ------------------------------------------------------ управление питанием
constexpr uint8_t PIN_PWR_REQ    = 3;    // SW1, INPUT_PULLUP: замкнут = работать
constexpr uint8_t PIN_KEEP_ALIVE = 7;    // R32 -> база Q7, держит питание робота
constexpr uint8_t PIN_MOT_PWR    = 39;   // R5 -> база Q3, питание моторов
constexpr uint8_t PIN_ESTOP      = 41;   // НО контакт грибка, INPUT_PULLUP

// ----------------------------------------------------------- силовые ключи
constexpr uint8_t PIN_FAN    = 8;        // ШИМ на затвор Q6
constexpr uint8_t PIN_BRAKE_F = 9;       // legacy F name: motor rail A, side must be measured
constexpr uint8_t PIN_BRAKE_R = 10;      // legacy R name: motor rail B, side must be measured

// -------------------------------------------------------------- индикация
constexpr uint8_t PIN_LED    = 6;        // R20 330 -> DIN первой ленты
constexpr uint8_t LED_COUNT  = 20;       // чипов WS2811, по 3 светодиода на чип
constexpr uint8_t LED_BRIGHT = 80;       // из 255; метр в белом на полную даёт 14 Вт

// ------------------------------------------------------------------ моторы
constexpr uint8_t MOTOR_COUNT = 8;
// STEP: D22..D29 — это целиком PORTA, бит i = мотор i.
// DIR:  D30..D37 — это целиком PORTC, но в обратном порядке: D30 = PC7.
// Такая раскладка позволяет дёргать все восемь шагов одной записью в порт.
constexpr uint8_t PIN_DRV_EN = 38;       // общий, низкий = драйверы разрешены

// Концевики, по одному на сустав. Второй вывод каждого — на землю.
const uint8_t PIN_LIMIT[MOTOR_COUNT] = { 42, 43, 44, 45, 46, 47, 48, 49 };

// --------------------------------------------------------------- драйверы
// Шина A (Serial1, выводы 18/19) — драйверы 0..3, адреса 0..3.
// Шина B (Serial2, выводы 16/17) — драйверы 4..7, адреса 0..3.
constexpr float    TMC_RSENSE     = 0.11f;   // MKS TMC2209 v2.0
constexpr uint32_t TMC_BAUD       = 115200;
constexpr uint16_t TMC_RUN_MA     = 600;     // initial supported-bench setting, RMS mA
constexpr float    TMC_HOLD_MULT  = 1.0f;    // avoid unverified loss of support at rest
constexpr uint16_t TMC_MICROSTEPS = 16;
constexpr uint16_t STEPS_PER_REV  = 200 * TMC_MICROSTEPS;   // 17HS4401 = 200 полных шагов

// --------------------------------------------------------- генератор шагов
constexpr uint16_t STEP_ISR_HZ   = 8000;    // initial 8 kHz, verify pulse timing on hardware
constexpr uint16_t SPEED_MAX_SPS = 1200;    // conservative initial ceiling
constexpr uint16_t HOMING_SPS    = 200;
constexpr uint16_t ACCEL_SPS2    = 1600;

// -------------------------------------------------------------- измерения
// Делители 100 кОм / 20 кОм: коэффициент 6.
// VREF — реальное напряжение шины 5 В, померь мультиметром.
constexpr uint16_t VREF_MV  = 5000;
constexpr uint8_t  DIV_MUL  = 6;
constexpr uint8_t  PIN_VBUS   = A0;      // шина батареи
constexpr uint8_t  PIN_V20F   = A1;      // motor rail A; verify physical side
constexpr uint8_t  PIN_V20R   = A2;      // motor rail B; verify physical side

// ------------------------------------------------------------ пороги 8S
constexpr uint16_t VBUS_WARN_MV  = 25000;   // жёлтая лента, флаг в телеметрию
constexpr uint16_t VBUS_DOWN_MV  = 24000;   // корректное выключение
constexpr uint16_t VBUS_SANE_MV  = 15000;   // ниже — считаем, что делитель оборван
constexpr uint16_t V20_BRAKE_MV  = 24000;   // выше — открыть тормозной ключ
constexpr uint16_t V20_BRAKE_OFF = 22000;   // гистерезис, чтобы ключ не дребезжал

// ---------------------------------------------------------------- тайминги
constexpr uint16_t LINK_BAUD_PI    = 38400;  // Serial3 к Orange Pi
constexpr uint16_t TM_PERIOD_MS    = 100;    // телеметрия 10 раз в секунду
constexpr uint16_t CTRL_TIMEOUT_MS = 500;    // нет команд — останавливаемся
constexpr uint16_t SENSE_PERIOD_MS = 50;     // опрос напряжений
constexpr uint16_t LED_PERIOD_MS   = 120;    // обновление ленты
constexpr uint32_t PI_SHUTDOWN_MS  = 60000;  // сколько ждём выключения Orange Pi
constexpr uint32_t POWER_OFF_MS    = 8000;   // пауза после этого до снятия питания
