#pragma once
#include <stdint.h>
#include <string.h>
struct Telemetry { uint16_t seq; uint8_t state; uint16_t mv; uint16_t flags; };
inline uint8_t packetCrc(const char* p, uint8_t length) {
  uint8_t c=0;
  for (uint8_t i=0;i<length;++i) {
    c^=(uint8_t)p[i];
    for (uint8_t b=0;b<8;++b) c=(c&0x80)?(uint8_t)((c<<1)^7):(uint8_t)(c<<1);
  }
  return c;
}
inline int8_t hexDigit(char c) {
  if(c>='0'&&c<='9') return c-'0';
  if(c>='A'&&c<='F') return c-'A'+10;
  if(c>='a'&&c<='f') return c-'a'+10;
  return -1;
}
inline bool readField(const char*& p, char delimiter, uint8_t base, uint16_t max, uint16_t& value) {
  uint32_t n=0; uint8_t count=0;
  while (*p && *p!=delimiter) {
    int8_t d=hexDigit(*p++);
    if(d<0 || d>=base) return false;
    n=n*base+d; if(n>max) return false;
    ++count;
  }
  if(!count || *p!=delimiter) return false;
  value=(uint16_t)n; if(delimiter) ++p;
  return true;
}
inline bool parseTelemetry(char* line, Telemetry& out) {
  char* star=strchr(line,'*');
  if(!star || strlen(star+1)!=2 || star-line>42) return false;
  const int8_t a=hexDigit(star[1]), b=hexDigit(star[2]);
  if(a<0 || b<0 || packetCrc(line,(uint8_t)(star-line))!=(uint8_t)(a*16+b)) return false;
  *star=0;
  const char* p=line;
  if(p[0]!='T'||p[1]!=',') return false;
  p+=2;
  Telemetry next; uint16_t state;
  if(!readField(p,',',10,65535,next.seq) || !readField(p,',',10,5,state) ||
     !readField(p,',',10,65535,next.mv) || !readField(p,0,16,65535,next.flags)) return false;
  next.state=(uint8_t)state; out=next; return true;
}
