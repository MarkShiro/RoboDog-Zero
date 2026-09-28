// Bench-only Pro Mini test: screen, eight buttons, local battery divider,
// and incoming HC-05 traffic. Never sends a movement or ARM command.
// Board: ATmega328P, 3.3 V, 8 MHz. ST7735 CS10/DC9/RST2.
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

Adafruit_ST7735 screen(10, 9, 2);
const uint8_t buttons[8] = {4, 5, 6, 7, 12, A0, A1, A2};
const char* names[8] = {"FWD", "BACK", "LEFT", "RIGHT", "MENU", "UP", "DOWN", "OK"};
uint32_t lastDraw = 0;
uint16_t rxLines = 0;

void setup() {
  for (uint8_t i=0; i<8; ++i) pinMode(buttons[i], INPUT_PULLUP);
  Serial.begin(38400);
  screen.initR(INITR_BLACKTAB);
  screen.setRotation(1);
  screen.fillScreen(ST77XX_BLACK);
}

void loop() {
  while (Serial.available()) if (Serial.read()=='\n') ++rxLines;
  if (millis()-lastDraw < 150) return;
  lastDraw=millis();
  screen.fillScreen(ST77XX_BLACK);
  screen.setTextSize(1);
  screen.setCursor(4,3);
  screen.setTextColor(ST77XX_CYAN);
  screen.print(F("REMOTE BENCH TEST"));
  for (uint8_t i=0; i<8; ++i) {
    const uint8_t row=i/2, col=i%2;
    const bool pressed=digitalRead(buttons[i])==LOW;
    const int16_t x=4+col*79, y=22+row*20;
    screen.fillRoundRect(x,y,75,17,3,pressed?ST77XX_GREEN:0x4208);
    screen.setTextColor(pressed?ST77XX_BLACK:ST77XX_WHITE);
    screen.setCursor(x+5,y+5); screen.print(names[i]);
  }
  const uint32_t mv=(uint32_t)analogRead(A6)*3300UL*156UL/(1023UL*56UL);
  screen.setTextColor(ST77XX_YELLOW);
  screen.setCursor(4,106); screen.print(F("LOCAL "));
  screen.print(mv/1000); screen.print('.');
  screen.print((mv%1000)/100); screen.print(F("V RX "));
  screen.print(rxLines);
}
