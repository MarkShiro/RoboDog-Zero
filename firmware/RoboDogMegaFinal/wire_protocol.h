#pragma once
#include <stdint.h>
#include <string.h>
inline uint8_t wireCrc(const char* s, uint8_t n) {
  uint8_t crc=0;
  while(n--) { crc^=(uint8_t)*s++; for(uint8_t b=0;b<8;++b) crc=crc&128?(uint8_t)((crc<<1)^7):(uint8_t)(crc<<1); }
  return crc;
}
inline int hexDigit(char c) {
  if(c>='0' && c<='9') return c-'0';
  if(c>='A' && c<='F') return c-'A'+10;
  if(c>='a' && c<='f') return c-'a'+10;
  return -1;
}
inline bool wireBody(char* line) {
  char* star=strchr(line,'*');
  if(!star || star==line || strlen(star+1)!=2) return false;
  const int a=hexDigit(star[1]),b=hexDigit(star[2]);
  if(a<0 || b<0 || wireCrc(line,(uint8_t)(star-line))!=(a*16+b)) return false;
  *star=0; return true;
}
inline bool wireNumber(const char*& p, uint8_t base, uint32_t max, uint16_t& out, char end) {
  uint32_t v=0; uint8_t n=0;
  while(*p && *p!=',') {
    int d=hexDigit(*p++);
    if(d<0 || d>=base || ++n>5) return false;
    v=v*base+d; if(v>max) return false;
  }
  if(!n || *p!=end) return false;
  if(end) ++p;
  out=(uint16_t)v; return true;
}
struct WireControl { uint16_t seq,buttons,remoteMv,speed,mode; };
inline bool parseControlBody(const char* p, WireControl& c) {
  if(*p++!='C' || *p++!=',') return false;
  return wireNumber(p,10,65535,c.seq,',') && wireNumber(p,16,1023,c.buttons,',') &&
    wireNumber(p,10,65535,c.remoteMv,',') && wireNumber(p,10,5,c.speed,',') &&
    wireNumber(p,10,255,c.mode,0);
}
inline bool sequenceNewer(uint16_t a,uint16_t b) {
  uint16_t delta=(uint16_t)(a-b); return delta && delta<32768;
}
