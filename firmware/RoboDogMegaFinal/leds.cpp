#include <Adafruit_NeoPixel.h>
#include "leds.h"
#include "steppers.h"

namespace {
Adafruit_NeoPixel strip(LED_COUNT, PIN_LED, NEO_GRB + NEO_KHZ800);

inline uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) {
  return strip.Color(r, g, b);
}
}  // namespace

void Leds::begin() {
  strip.begin();
  strip.setBrightness(LED_BRIGHT);
  strip.clear();
  strip.show();
}

void Leds::set(LedMode m) {
  if (m == _mode) return;
  _mode  = m;
  _phase = 0;
  _dirty = true;
}

void Leds::update(bool motorsMoving) {
  if(Steppers::enabled()) return; // NeoPixel disables IRQs: never disturb STEP timing
  const uint32_t now = millis();
  if ((int32_t)(now - _next) < 0) return;

  // Пока моторы идут, обновляем ленту вчетверо реже: каждая передача
  // отнимает у генератора шагов около полумиллисекунды.
  _next = now + (motorsMoving ? LED_PERIOD_MS * 4 : LED_PERIOD_MS);

  const bool animated = (_mode == LED_BOOT || _mode == LED_SHUTDOWN ||
                         _mode == LED_FAULT);
  if (!animated && !_dirty) return;
  _dirty = false;
  _phase++;

  switch (_mode) {
    case LED_BOOT: {
      strip.clear();
      const uint8_t head = _phase % LED_COUNT;
      for (uint8_t i = 0; i < 4; i++) {
        const uint8_t p = (uint8_t)((head + LED_COUNT - i) % LED_COUNT);
        strip.setPixelColor(p, rgb(0, 0, (uint8_t)(255 >> i)));
      }
      break;
    }
    case LED_OK:       strip.fill(rgb(0, 180, 40));  break;
    case LED_WARN:     strip.fill(rgb(220, 160, 0)); break;
    case LED_ESTOP:    strip.fill(rgb(255, 0, 0));   break;
    case LED_SHUTDOWN:
      strip.fill((_phase & 1) ? rgb(220, 160, 0) : rgb(0, 0, 0));
      break;
    case LED_FAULT:
      strip.fill((_phase & 1) ? rgb(255, 0, 0) : rgb(0, 0, 0));
      break;
    case LED_OFFREADY:
      strip.clear();
      strip.setPixelColor(0, rgb(0, 180, 40));
      break;
  }
  strip.show();
}
