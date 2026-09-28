#pragma once
#include <stdint.h>

// 8S LiFePO4. Provisional VOLTAGE scale inherited from the project.
// 0% = robot cutoff 24.0 V; 100% = 26.6 V, NOT the charger endpoint.
// LiFePO4 has a flat discharge curve: ~% is not measured state of charge.
// Calibrate these endpoints against your pack at rest; load sag changes ~%.
constexpr uint16_t BATTERY_EMPTY_MV = 24000;
constexpr uint16_t BATTERY_FULL_MV = 26600;
static_assert(BATTERY_FULL_MV > BATTERY_EMPTY_MV, "Invalid battery range");
constexpr uint8_t batteryPercent(uint16_t mv) {
  return mv <= BATTERY_EMPTY_MV ? 0 : mv >= BATTERY_FULL_MV ? 100 :
    (uint32_t)(mv - BATTERY_EMPTY_MV) * 100UL / (BATTERY_FULL_MV - BATTERY_EMPTY_MV);
}
// Unambiguous thresholds: 0..29 red, 30..59 yellow, 60..99 green, 100 blue.
constexpr uint8_t batteryBand(uint8_t pct) {
  return pct < 30 ? 0 : pct < 60 ? 1 : pct < 100 ? 2 : 3;
}
static_assert(batteryPercent(0)==0 && batteryPercent(23999)==0,"clamp low");
static_assert(batteryPercent(24000)==0 && batteryPercent(25300)==50,"voltage scale");
static_assert(batteryPercent(26600)==100 && batteryPercent(29200)==100,"clamp high");
static_assert(batteryBand(29)==0 && batteryBand(30)==1,"red/yellow boundary");
static_assert(batteryBand(59)==1 && batteryBand(60)==2,"yellow/green boundary");
static_assert(batteryBand(99)==2 && batteryBand(100)==3,"green/blue boundary");
