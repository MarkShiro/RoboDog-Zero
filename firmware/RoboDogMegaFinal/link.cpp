#include "link.h"
#include "wire_protocol.h"
uint8_t Link::crc8(const char* s,uint8_t n) { return wireCrc(s,n); }
void Link::begin() { Serial3.begin(LINK_BAUD_PI); _nextTx=millis(); }
void Link::poll() {
  uint8_t budget=96;
  while(budget-- && Serial3.available()) {
    char ch=(char)Serial3.read();
    if(ch=='\r') continue;
    if(ch=='\n') {
      if(!_overflow && _len) { _buf[_len]=0; handleLine(); }
      _len=0; _overflow=false;
    } else if(!_overflow) {
      if(ch<32 || ch>126 || _len>=sizeof(_buf)-1) { _overflow=true; _len=0; }
      else _buf[_len++]=ch;
    }
  }
}
void Link::handleLine() {
  if(!wireBody(_buf)) { ++_crcErr; return; }
  if(_buf[0]=='A' && _buf[1]==',') {
    const char* p=_buf+2; uint16_t seq;
    if(wireNumber(p,10,65535,seq,0) && fresh() && _c.buttons==0 &&
       seq==_controlSeq && (!_armSeen || seq!=_armSeq)) {
      _arm=true; _armSeen=true; _armSeq=seq;
    }
    return;
  }
  WireControl c;
  if(!parseControlBody(_buf,c)) return;
  if(c.buttons & CMD_STOP) { _stop=true; _arm=false; }
  if(fresh() && !sequenceNewer(c.seq,_controlSeq)) return;
  _controlSeq=c.seq;
  _c.buttons=c.buttons; _c.remoteMv=c.remoteMv;
  _c.speed=(uint8_t)c.speed; _c.mode=(uint8_t)c.mode;
  if(c.buttons) _arm=false;
  ++_rx; _lastRx=millis(); _ever=true;
}
bool Link::fresh() const { return _ever && (uint32_t)(millis()-_lastRx)<CTRL_TIMEOUT_MS; }
void Link::sendTelemetry(uint8_t state,uint16_t mv,uint16_t flags) {
  const uint32_t now=millis();
  if((int32_t)(now-_nextTx)<0) return;
  _nextTx=now+TM_PERIOD_MS;
  char body[40],line[48];
  int n=snprintf(body,sizeof(body),"T,%u,%u,%u,%04X",(unsigned)_seq++,(unsigned)state,(unsigned)mv,(unsigned)flags);
  if(n<=0 || n>=(int)sizeof(body)) return;
  snprintf(line,sizeof(line),"%s*%02X\n",body,wireCrc(body,(uint8_t)n)); Serial3.print(line);
}
void Link::sendShutdown() {
  char line[8]; snprintf(line,sizeof(line),"X*%02X\n",wireCrc("X",1)); Serial3.print(line);
}
void Link::sendImu(int16_t rollDeg,int16_t pitchDeg,int16_t tempC,uint16_t flags) {
  const uint32_t now=millis();
  if((int32_t)(now-_nextImu)<0) return;
  _nextImu=now+200; // 5 Hz display data, independent of 10 Hz T
  char body[48],line[56];
  const int n=snprintf(body,sizeof(body),"I,%d,%d,%d,%04X",
    (int)rollDeg,(int)pitchDeg,(int)tempC,(unsigned)flags);
  if(n<=0 || n>=(int)sizeof(body)) return;
  snprintf(line,sizeof(line),"%s*%02X\n",body,wireCrc(body,(uint8_t)n));
  Serial3.print(line);
}
