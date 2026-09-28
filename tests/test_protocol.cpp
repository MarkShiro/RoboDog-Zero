#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/RoboDogRemoteFinal/telemetry.h"
#include "../firmware/RoboDogRemoteFinal/battery.h"
#include "../firmware/RoboDogMotorMap/bench_limits.h"
static bool parse(const char* body) {
  char line[128];
  snprintf(line,sizeof(line),"%s*%02X",body,packetCrc(body,(uint8_t)strlen(body)));
  Telemetry t={}; return parseTelemetry(line,t);
}
int main() {
  const char* body="T,65535,2,25600,000F";
  char line[128]; snprintf(line,sizeof(line),"%s*%02X",body,packetCrc(body,strlen(body)));
  Telemetry t={}; assert(parseTelemetry(line,t));
  assert(t.seq==65535 && t.state==2 && t.mv==25600 && t.flags==15);
  assert(parse("T,0,0,0,0000"));
  const char* invalid[]={"C,1,2,25600,000F","TT,1,2,25600,000F","T,1,6,25600,000F",
    "T,65536,2,25600,000F","T,1,2,-1,000F","T,1,2,25600","T,1,2,25600,",
    "T,,2,25600,000F","T,1,2,25600,000F,3","T,1,2,25600,10000",
    "T,1,2,25600x,000F","T,1,2,65536,000F","T,1,2,25600,00QF"};
  for(auto s:invalid) assert(!parse(s));
  char badCrc[]="T,1,1,25000,000F*ZZ"; assert(!parseTelemetry(badCrc,t));
  char shortCrc[]="T,1,1,25000,000F*A"; assert(!parseTelemetry(shortCrc,t));
  char extra[]="T,1,1,25000,000F*000"; assert(!parseTelemetry(extra,t));
  snprintf(line,sizeof(line),"%s*%02X",body,packetCrc(body,strlen(body))^1);
  assert(!parseTelemetry(line,t));
  uint8_t prev=0;
  for(unsigned mv=0;mv<=65535;++mv) {
    uint8_t pct=batteryPercent(mv); assert(pct<=100 && pct>=prev); prev=pct;
  }
  assert(batteryPercent(24780)==30 && batteryPercent(25560)==60);
  for(int p=-500;p<=500;++p) for(int n=-101;n<=101;++n) {
    bool ok=benchJogAllowed(p,900,n);
    if(ok) assert(n && n>=-100 && n<=100 && p+n>=-500 && p+n<=500);
  }
  puts("PASS: telemetry fields/CRC, all voltage inputs, color and jog boundaries");
}
