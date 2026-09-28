#pragma once
#include "Arduino.h"
#include <assert.h>
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_CYAN 0x07FF
#define ST77XX_GREEN 0x07E0
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_RED 0xF800
#define ST77XX_BLUE 0x001F
#define INITR_BLACKTAB 0
class Adafruit_ST7735 {
 public:
 uint16_t pixels[160*128]={};
 Adafruit_ST7735(int,int,int) {}
 void initR(int) {} void setRotation(int) {}
 void drawPixel(int x,int y,uint16_t c) {
   assert(x>=0 && x<160 && y>=0 && y<128); pixels[y*160+x]=c;
 }
 void fillRect(int x,int y,int w,int h,uint16_t c) {
   for(int j=y;j<y+h;++j) for(int i=x;i<x+w;++i) drawPixel(i,j,c);
 }
 void fillScreen(uint16_t c) { fillRect(0,0,160,128,c); }
 void drawRect(int x,int y,int w,int h,uint16_t c) {
   fillRect(x,y,w,1,c); fillRect(x,y+h-1,w,1,c);
   fillRect(x,y,1,h,c); fillRect(x+w-1,y,1,h,c);
 }
 void save(const char* path) {
   FILE* f=fopen(path,"wb"); assert(f);
   fprintf(f,"P6\n160 128\n255\n");
   for(auto c:pixels) { unsigned char rgb[3]={
     (unsigned char)(((c>>11)&31)*255/31),
     (unsigned char)(((c>>5)&63)*255/63),
     (unsigned char)((c&31)*255/31)}; fwrite(rgb,1,3,f); }
   fclose(f);
 }
};
