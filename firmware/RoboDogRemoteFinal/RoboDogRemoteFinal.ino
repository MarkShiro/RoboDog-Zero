// =====================================================================
//  RoboDogRemoteFinal — пульт с оценкой заряда 8S LiFePO4.
//
//  Плата:      Arduino Pro Mini, ATmega328P, 3,3 В / 8 МГц
//              (в IDE: «Arduino Pro or Pro Mini» -> ATmega328P 3.3V 8MHz)
//  Экран:      ST7735 1,77" 128x160 на аппаратном SPI
//  Связь:      HC-05 на выводах D0 и D1, 38400 бод
//  Библиотеки: Adafruit GFX Library, Adafruit ST7735 and ST7789 Library
//
//  Перед прошивкой снять джампер X2 в линии TXD модуля HC-05.
//
//  ---- КНОПКИ --------------------------------------------------------
//    1  D4    вперёд
//    2  D5    назад
//    3  D6    влево
//    4  D7    вправо
//    5  D12   меню: открыть или закрыть
//    6  A0    вверх по меню
//    7  A1    вниз по меню
//    8  A2    подтвердить
//
//  Кнопки 1-4 работают только при закрытом меню, кнопки 6-8 — только
//  при открытом. Так одно нажатие никогда не значит двух вещей сразу.
// =====================================================================
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "cyr_font.h"
#include "battery.h"
#include "telemetry.h"

// --------------------------------------------------------------- выводы
#define TFT_CS   10
#define TFT_DC    9
#define TFT_RES   2
#define VBAT_PIN A6          // делитель напряжения батареи пульта

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RES);

const uint8_t BTN_PIN[8] = { 4, 5, 6, 7, 12, A0, A1, A2 };

// Номера кнопок, чтобы дальше в коде читались словами
#define B_FWD   0
#define B_BACK  1
#define B_LEFT  2
#define B_RIGHT 3
#define B_MENU  4
#define B_UP    5
#define B_DOWN  6
#define B_OK    7

// --------------------------------------------------------------- цвета
#define C_BG     ST77XX_BLACK
#define C_TEXT   ST77XX_WHITE
#define C_DIM    0x8410
#define C_TITLE  ST77XX_CYAN
#define C_OK     ST77XX_GREEN
#define C_WARN   ST77XX_YELLOW
#define C_BAD    ST77XX_RED
#define C_SEL    0x2124        // подсветка выбранной строки меню

// ------------------------------------------------------------ настройки
#define LINK_BAUD     38400
#define SEND_PERIOD    50      // отправка команд, миллисекунды
#define DRAW_PERIOD   150      // перерисовка экрана
#define LINK_TIMEOUT 1500      // нет ответа дольше — считаем, что связи нет
#define DEBOUNCE       25      // фильтр дребезга кнопок
#define LAY_HOLD      800      // сколько держать команду «лечь»

// Пороги напряжения робота, милливольты (сборка 8S LiFePO4)
// Оценка по напряжению настроена в battery.h; это не счётчик ёмкости.

// ------------------------------------------------------- состояние кнопок
bool     btnRaw[8], btnHeld[8], btnPressed[8];
uint32_t btnChanged[8];

// --------------------------------------------------------- состояние меню
bool     menuOpen  = false;
uint8_t  menuItem  = 0;        // 0 — лечь, 1 — моторы
bool     motorsOff = false;    // команду держим, пока пункт включён
uint32_t layStarted = 0;
bool layActive = false;

// ------------------------------------------------- что пришло от робота
uint16_t robotMv = 0;
uint32_t lastRx  = 0;
char     rxBuf[48];
uint8_t  rxLen   = 0;
bool rxOverflow = false, everRx = false;
uint16_t robotFlags = 0;
uint8_t robotState = 0;

// ------------------------------------------------------------- служебное
uint16_t seq = 0;
uint32_t tSend = 0, tDraw = 0;

// Что было нарисовано в прошлый раз — чтобы не перерисовывать зря
int8_t   drawnMode = -1;       // -1 ничего, 0 главный экран, 1 меню
uint8_t  drawnItem = 255;
uint16_t drawnMv   = 0xFFFF;
uint16_t drawnFlags = 0xFFFF;
uint8_t drawnState = 255;
bool     drawnLink = false, drawnMotors = false;

bool linkAlive() { return everRx && (millis() - lastRx) < LINK_TIMEOUT; }

// =====================================================================
//  ТЕКСТ. Встроенный шрифт Adafruit кириллицы не знает, поэтому буквы
//  рисуем сами по таблице из cyr_font.h.
// =====================================================================

// Достаёт из строки очередной символ. Русская буква занимает два байта,
// латинская — один; функция сама разбирается и сдвигает указатель.
uint16_t nextChar(const char** p) {
  uint8_t b = (uint8_t)*(*p)++;
  if (b < 0x80) return b;
  if ((b & 0xE0) == 0xC0) {
    uint8_t b2 = (uint8_t)*(*p);
    if (b2) { (*p)++; return ((uint16_t)(b & 0x1F) << 6) | (b2 & 0x3F); }
  }
  return '?';
}

// Номер картинки буквы в таблице
uint8_t glyphOf(uint16_t cp) {
  if (cp >= FONT_ASCII_FIRST && cp <= FONT_ASCII_LAST) return cp - FONT_ASCII_FIRST;
  if (cp >= FONT_CYR_FIRST   && cp <= FONT_CYR_LAST)   return FONT_CYR_INDEX + (cp - FONT_CYR_FIRST);
  if (cp == 0x401) return FONT_CYR_INDEX + (0x415 - FONT_CYR_FIRST);   // Ё как Е
  if (cp == 0x451) return FONT_CYR_INDEX + (0x435 - FONT_CYR_FIRST);   // ё как е
  return 0;                                                            // всё прочее — пробел
}

// Ширина строки в точках экрана
int16_t textWidth(const char* s, uint8_t size) {
  int16_t n = 0;
  while (*s) { nextChar(&s); n++; }
  return n * FONT_W * size;
}

void drawText(int16_t x, int16_t y, const char* s, uint16_t color, uint8_t size) {
  while (*s) {
    const uint8_t g = glyphOf(nextChar(&s));
    for (uint8_t c = 0; c < FONT_W; c++) {
      const uint8_t bits = pgm_read_byte(&FONT6x8[(uint16_t)g * FONT_W + c]);
      for (uint8_t r = 0; r < FONT_H; r++) {
        if (!(bits & (1 << r))) continue;
        if (size == 1) tft.drawPixel(x + c, y + r, color);
        else           tft.fillRect(x + c * size, y + r * size, size, size, color);
      }
    }
    x += FONT_W * size;
  }
}

void drawTextCenter(int16_t y, const char* s, uint16_t color, uint8_t size) {
  drawText((160 - textWidth(s, size)) / 2, y, s, color, size);
}

// =====================================================================
//  КНОПКИ
// =====================================================================
void buttonsSetup() {
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(BTN_PIN[i], INPUT_PULLUP);
    btnRaw[i] = btnHeld[i] = false;
    btnChanged[i] = 0;
  }
}

void buttonsUpdate() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < 8; i++) {
    btnPressed[i] = false;
    const bool down = (digitalRead(BTN_PIN[i]) == LOW);   // нажата = ноль
    if (down != btnRaw[i]) {                              // уровень дёрнулся
      btnRaw[i] = down;
      btnChanged[i] = now;
      continue;                                           // ждём, пока успокоится
    }
    if (now - btnChanged[i] < DEBOUNCE) continue;
    if (down && !btnHeld[i]) btnPressed[i] = true;         // подтверждённое нажатие
    btnHeld[i] = down;
  }
}

// =====================================================================
//  СВЯЗЬ
// =====================================================================
uint8_t crc8(const char* s, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= (uint8_t)s[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

void sendPacket() {
  uint16_t bits = 0;
  if (!menuOpen) {                       // движение — только при закрытом меню
    if (btnHeld[B_FWD])   bits |= 1 << 0;
    if (btnHeld[B_BACK])  bits |= 1 << 1;
    if (btnHeld[B_LEFT])  bits |= 1 << 2;
    if (btnHeld[B_RIGHT]) bits |= 1 << 3;
  }
  if (layActive && millis() - layStarted >= LAY_HOLD) layActive = false;
  if (layActive) bits |= 1 << 4;   // «лечь»
  if (motorsOff)           bits |= 1 << 9;   // «моторы обесточены»

  const uint16_t ownMv = (uint16_t)((uint32_t)analogRead(VBAT_PIN) * 3300 / 1023 * 156 / 56);

  char body[40];
  const int n = snprintf(body, sizeof(body), "C,%u,%04X,%u,2,0",
                         (unsigned)seq++, (unsigned)bits, (unsigned)ownMv);
  if (n <= 0 || n >= (int)sizeof(body)) return;

  char line[48];
  snprintf(line, sizeof(line), "%s*%02X\n", body, crc8(body, (uint8_t)n));
  Serial.print(line);
}

void readPacket() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (rxOverflow) continue;
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else { rxOverflow = true; rxLen = 0; }
      continue;
    }
    if (rxOverflow) { rxOverflow = false; rxLen = 0; continue; }
    rxBuf[rxLen] = '\0';
    rxLen = 0;
    Telemetry tm;
    if (!parseTelemetry(rxBuf, tm)) continue;
    robotMv = tm.mv;
    robotFlags = tm.flags;
    robotState = tm.state;
    lastRx = millis();
    everRx = true;
  }
}

// =====================================================================
//  ЭКРАН
// =====================================================================
uint16_t batColor(uint8_t pct) {
  switch (batteryBand(pct)) {
    case 0: return C_BAD;
    case 1: return C_WARN;
    case 2: return C_OK;
    default: return ST77XX_BLUE;
  }
}

void drawBattery() {
  const bool live = linkAlive();
  const bool valid = live && !(robotFlags & (1 << 7)) && robotMv > 15000 && robotMv <= 29500;
  char s[20];
  tft.fillRect(0, 0, 160, 27, C_BG);
  if (valid) {
    const uint8_t pct = batteryPercent(robotMv);
    const uint16_t color = batColor(pct);
    snprintf(s, sizeof(s), "%u.%u В ~%u%%", robotMv / 1000,
             (robotMv % 1000) / 100, (unsigned)pct);
    drawText(4, 2, s, color, 1);
    tft.drawRect(4, 15, 70, 7, C_DIM);
    if (pct) tft.fillRect(5, 16, (uint16_t)68 * pct / 100, 5, color);
  } else {
    drawText(4, 2, "--.- В --%", C_DIM, 1);
    tft.drawRect(4, 15, 70, 7, C_DIM);
  }
  const char* lbl = live ? "СВЯЗЬ" : "НЕТ СВЯЗИ";
  drawText(156 - textWidth(lbl, 1), 15, lbl, live ? C_OK : C_BAD, 1);
  drawnMv = robotMv;
  drawnLink = live;
}

void drawStatus() {
  tft.fillRect(0, 62, 160, 35, C_BG);
  const char* text = "ОЖИДАНИЕ РОБОТА";
  uint16_t color = C_DIM;
  if (linkAlive()) {
    if (robotState == 4) { text = "АВАРИЙНЫЙ СТОП"; color = C_BAD; }
    else if (robotState == 5) { text = "ВЫКЛЮЧЕНИЕ"; color = C_WARN; }
    else if (robotState == 0) { text = "ПОИСК НУЛЕЙ"; color = C_WARN; }
    else if (!(robotFlags & 1)) { text = "ОШИБКА ДРАЙВЕРОВ"; color = C_BAD; }
    else if (!(robotFlags & (1 << 3))) { text = "МОТОРЫ ВЫКЛЮЧЕНЫ"; color = C_WARN; }
    else if (robotFlags & (1 << 6)) { text = "НЕТ КОМАНД"; color = C_WARN; }
    else if (!(robotFlags & (1 << 2))) { text = "НУЛИ НЕ НАЙДЕНЫ"; color = C_WARN; }
    else if (robotFlags & (1 << 5)) { text = "НИЗКИЙ ЗАРЯД"; color = C_WARN; }
    else { text = "РОБОТ ГОТОВ"; color = C_OK; }
  }
  drawTextCenter(64, text, color, 1);
  if (motorsOff) drawTextCenter(80, "ЗАПРОС: МОТОРЫ ВЫКЛ", C_WARN, 1);
  drawnState = robotState;
  drawnFlags = robotFlags;
}

void drawMainScreen() {
  tft.fillScreen(C_BG);
  drawBattery();
  drawTextCenter(32, "RoboDog Zero", C_TITLE, 2);

  drawStatus();

  drawText(4, 116, "5 - МЕНЮ", C_DIM, 1);
  drawnMode = 0;
  drawnMotors = motorsOff;
}

void drawMenuScreen() {
  tft.fillScreen(C_BG);
  drawBattery();
  drawTextCenter(32, "RoboDog Zero", C_TITLE, 2);

  tft.drawRect(6, 54, 148, 70, C_DIM);

  // Пункт 1
  if (menuItem == 0) tft.fillRect(8, 58, 144, 20, C_SEL);
  drawText(14, 60, menuItem == 0 ? ">" : " ", C_TITLE, 2);
  drawText(30, 60, "ЛЕЧЬ", C_TEXT, 2);

  // Пункт 2 — в две строки, чтобы влезло целиком
  if (menuItem == 1) tft.fillRect(8, 82, 144, 38, C_SEL);
  drawText(14, 84, menuItem == 1 ? ">" : " ", C_TITLE, 2);
  if (motorsOff) {
    drawText(30, 84,  "ВКЛЮЧИТЬ", C_TEXT, 2);
    drawText(30, 102, "МОТОРЫ", C_TEXT, 2);
  } else {
    drawText(30, 84,  "ВЫКЛЮЧИТЬ", C_TEXT, 2);
    drawText(30, 102, "МОТОРЫ", C_TEXT, 2);
  }

  drawnMode = 1;
  drawnItem = menuItem;
  drawnMotors = motorsOff;
}

void redraw() {
  const int8_t mode = menuOpen ? 1 : 0;
  const bool changed = (mode != drawnMode) || (menuOpen && menuItem != drawnItem) ||
                       (motorsOff != drawnMotors);
  if (changed) {
    if (mode == 0) drawMainScreen(); else drawMenuScreen();
    return;
  }
  const bool linkChanged = linkAlive() != drawnLink;
  if (robotMv != drawnMv || robotFlags != drawnFlags || linkChanged) drawBattery();
  if (!menuOpen && (robotState != drawnState || robotFlags != drawnFlags || linkChanged)) drawStatus();
  if (menuOpen) drawnFlags = robotFlags;
}

// =====================================================================
//  ЗАПУСК И ОСНОВНОЙ ЦИКЛ
// =====================================================================
void setup() {
  buttonsSetup();
  Serial.begin(LINK_BAUD);

  tft.initR(INITR_BLACKTAB);       // не та картинка — попробуй GREENTAB или REDTAB
  tft.setRotation(1);              // альбомная ориентация, 160 x 128
  tft.fillScreen(C_BG);

  drawTextCenter(50, "RoboDog Zero", C_TITLE, 2);
  drawTextCenter(74, "ЗАГРУЗКА...", C_DIM, 1);
  delay(1200);                     // за это время HC-05 успевает подняться

  drawMainScreen();
}

void loop() {
  buttonsUpdate();
  readPacket();

  // ------------------------------------------------------- меню
  if (btnPressed[B_MENU]) menuOpen = !menuOpen;

  if (menuOpen) {
    if (btnPressed[B_UP]   && menuItem > 0) menuItem--;
    if (btnPressed[B_DOWN] && menuItem < 1) menuItem++;
    if (btnPressed[B_OK]) {
      if (menuItem == 0) { layStarted = millis(); layActive = true; }   // лечь
      else               motorsOff = !motorsOff;           // моторы вкл/выкл
      menuOpen = false;                                    // и закрыть меню
    }
  }

  // ------------------------------------------------------- связь
  if (millis() - tSend >= SEND_PERIOD) { tSend = millis(); sendPacket(); }

  // ------------------------------------------------------- экран
  if (millis() - tDraw >= DRAW_PERIOD) { tDraw = millis(); redraw(); }
}
