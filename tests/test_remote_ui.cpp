#include <assert.h>
#include <stdio.h>
#include "../firmware/RoboDogRemoteFinal/RoboDogRemoteFinal.ino"
void telemetry(const char* body) {
 char line[80]; snprintf(line,sizeof(line),"%s*%02X\n",body,packetCrc(body,strlen(body)));
 Serial.feed(line); readPacket();
}
int main() {
 setup(); assert(!linkAlive());
 telemetry("T,1,1,25600,000F"); assert(linkAlive() && robotMv==25600);
 const uint32_t goodRx=lastRx;
 Serial.feed("T,1,1,28000,000F*ZZ\n"); readPacket(); assert(robotMv==25600 && lastRx==goodRx);
 Serial.feed("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n"); readPacket();
 assert(robotMv==25600);
 const uint16_t volts[]={24520,25170,26080,26600};
 const char* paths[]={"tmp/inspection/remote-red.ppm","tmp/inspection/remote-yellow.ppm",
   "tmp/inspection/remote-green.ppm","tmp/inspection/remote-blue.ppm"};
 for(int i=0;i<4;++i) { robotMv=volts[i]; drawMainScreen(); tft.save(paths[i]); }
 menuOpen=true; menuItem=1; motorsOff=false; drawMenuScreen();
 tft.save("tmp/inspection/remote-menu.ppm");
 menuOpen=false; clockUs+=1501000; redraw(); assert(!linkAlive());
 tft.save("tmp/inspection/remote-offline.ppm");
 puts("PASS: remote receive/overflow, screen bounds, main/menu/offline rendering");
}
