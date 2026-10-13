/*
 Pops ESP32-S3 grass-milling navigation prototype v0.1
 Safe commissioning build: NO autonomous DRV commands.
 Cart Rev10 UART: cart TX GPIO17 -> S3 RX GPIO18;
 S3 TX GPIO21 -> cart RX GPIO16; common 3.3V GND.
 WARNING: BU04 binary packets were observed on USB, NOT proven on TTL.
 UWB UART GPIO16 RX / GPIO17 TX is an EXPERIMENTAL input only.
 Serial Monitor uses UART0 on the S3 COM connector (USB CDC On Boot OFF).
*/
#include <Arduino.h>
#include <math.h>

namespace Config {
constexpr int CART_RX=18, CART_TX=21, UWB_RX=16, UWB_TX=17;
constexpr uint32_t BAUD=115200;
constexpr float CART_WIDTH_MM=1016.0f, CART_LENGTH_MM=1016.0f;
constexpr float WHEELBASE_MM=889.0f;
constexpr float CUT_WIDTH_MM=508.0f, CUT_LENGTH_MM=508.0f;
constexpr float FRONT_TO_BU04_MM=228.6f; // configurable initial 9 inches
constexpr float CUT_OVERLAP_MM=50.8f;
constexpr float CLEARANCE_MM=101.6f; // illustrative, NOT certified safe
constexpr uint32_t RANGE_MAX_AGE_MS=600;
constexpr uint32_t POSE_MAX_AGE_MS=800;
constexpr uint32_t STOP_PERIOD_MS=100;
constexpr bool ENABLE_MOTION=false; // commissioning lock: never send DRV
}
HardwareSerial cart(1), uwb(2);
struct Vec2 {float x,y;};
struct Anchor {Vec2 pos; float range; uint32_t stamp; bool valid;};
// These are UNCONFIGURED until real surveyed coordinates are entered.
Anchor anchors[4]={};
bool anchorsConfigured=false;
struct Pose {Vec2 antenna; Vec2 center; float heading; float residual; uint32_t stamp; bool valid;} pose={};
bool headingConfigured=false;
enum class NavState {SAFE_STOP, LEARN_BOUNDARY, READY, BLOCKED, SENSOR_FAULT};
NavState state=NavState::SAFE_STOP;
constexpr int MAX_BOUNDARY=256;
Vec2 boundary[MAX_BOUNDARY]; int boundaryCount=0;
constexpr int MAX_OBSTACLES=128;
struct Obstacle {Vec2 pos; float distanceMm; uint8_t side; uint32_t timestamp;};
Obstacle obstacles[MAX_OBSTACLES]; int obstacleCount=0;
char cartLine[100]={}, usbLine[100]={};
size_t cartLen=0, usbLen=0;
uint32_t lastStop=0, lastPrint=0;
bool frontBlocked=false, rearBlocked=false, sensorFault=false;
uint32_t uwbPackets=0, uwbBad=0;

// A reported UWB distance is not a position; use at least 3 fresh ranges.
// 2D Gauss-Newton weighted least-squares, initialized at anchor centroid.
bool solvePosition(uint32_t now,Vec2 &out,float &rms){
 if(!anchorsConfigured)return false;
 int n=0;Vec2 p={0,0};
 for(int i=0;i<4;i++)if(anchors[i].valid && now-anchors[i].stamp<Config::RANGE_MAX_AGE_MS &&
 anchors[i].range>100 && anchors[i].range<100000){p.x+=anchors[i].pos.x;p.y+=anchors[i].pos.y;n++;}
 if(n<3)return false;p.x/=n;p.y/=n;
 for(int iter=0;iter<18;iter++){
  float aa=0,ab=0,bb=0,ac=0,bc=0;
  for(int i=0;i<4;i++){
   Anchor &a=anchors[i];if(!a.valid||now-a.stamp>=Config::RANGE_MAX_AGE_MS||a.range<=100||a.range>=100000)continue;
   float dx=p.x-a.pos.x,dy=p.y-a.pos.y,d=hypotf(dx,dy);if(d<1) return false;
   float ex=d-a.range,jx=dx/d,jy=dy/d;
   aa+=jx*jx;ab+=jx*jy;bb+=jy*jy;ac+=jx*ex;bc+=jy*ex;
  }
  float det=aa*bb-ab*ab;if(fabsf(det)<0.00001f)return false;
  float sx=(bb*ac-ab*bc)/det,sy=(aa*bc-ab*ac)/det;
  p.x-=sx;p.y-=sy;if(hypotf(sx,sy)<0.5f)break;
 }
 float err=0;for(int i=0;i<4;i++){
  Anchor &a=anchors[i];if(!a.valid||now-a.stamp>=Config::RANGE_MAX_AGE_MS||a.range<=100||a.range>=100000)continue;
  float e=hypotf(p.x-a.pos.x,p.y-a.pos.y)-a.range;err+=e*e;
 }
 rms=sqrtf(err/n);if(!isfinite(rms)||rms>500)return false;
 out=p;return true;
}
Vec2 rotateOffset(Vec2 offset,float theta){
 return {offset.x*cosf(theta)-offset.y*sinf(theta),
         offset.x*sinf(theta)+offset.y*cosf(theta)};
}
// Heading radians: +X forward, +Y left; BU04 is forward of cart center.
Vec2 cartCenter(Vec2 antenna,float theta){
 Vec2 v=rotateOffset({-(Config::CART_LENGTH_MM/2-Config::FRONT_TO_BU04_MM),0},theta);
 return {antenna.x+v.x,antenna.y+v.y};
}
bool pointInPolygon(Vec2 p){
 if(boundaryCount<3)return false;bool inside=false;
 for(int i=0,j=boundaryCount-1;i<boundaryCount;j=i++){
  Vec2 a=boundary[i],b=boundary[j];
  if((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x)inside=!inside;
 }return inside;
}
float pointSegmentDistance(Vec2 p,Vec2 a,Vec2 b){
 float dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
 float t=den>0?((p.x-a.x)*dx+(p.y-a.y)*dy)/den:0;
 t=fmaxf(0,fminf(1,t));return hypotf(p.x-(a.x+t*dx),p.y-(a.y+t*dy));
}
bool footprintInside(Vec2 center,float heading){
 if(boundaryCount<3)return false;
 // Corners and center must be inside, plus distance-to-boundary margin.
 Vec2 local[5]={{0,0},{508,508},{508,-508},{-508,508},{-508,-508}};
 for(auto v:local){
  Vec2 r=rotateOffset(v,heading),p={center.x+r.x,center.y+r.y};
  if(!pointInPolygon(p))return false;
  for(int i=0;i<boundaryCount;i++)
   if(pointSegmentDistance(p,boundary[i],boundary[(i+1)%boundaryCount])<Config::CLEARANCE_MM)return false;
 }return true;
}
void sendStop(){cart.print("STOP\n");}
void updatePose(){
 uint32_t now=millis();Vec2 v;float rms;
 if(!solvePosition(now,v,rms)){pose.valid=false;return;}
 pose.antenna=v;pose.residual=rms;pose.stamp=now;
 pose.valid=headingConfigured; // never claim full cart pose without heading
 if(headingConfigured)pose.center=cartCenter(v,pose.heading);
}
void recordObstacle(uint8_t side,float mm){
 if(!pose.valid||obstacleCount>=MAX_OBSTACLES)return;
 // Sonar at cart FRONT/REAR; direction of travel is not assumed.
 float longitudinal=(side==0?Config::CART_LENGTH_MM/2:-Config::CART_LENGTH_MM/2)+(side==0?mm:-mm);
 Vec2 d=rotateOffset({longitudinal,0},pose.heading);
 obstacles[obstacleCount++]={{pose.center.x+d.x,pose.center.y+d.y},mm,side,millis()};
}
void handleCartLine(const char *line){
 if(strncmp(line,"OBS,FRONT,",10)==0){frontBlocked=true;state=NavState::BLOCKED;recordObstacle(0,atof(line+10));}
 else if(strncmp(line,"OBS,REAR,",9)==0){rearBlocked=true;state=NavState::BLOCKED;recordObstacle(1,atof(line+9));}
 else if(strncmp(line,"CLEAR,FRONT,",12)==0){frontBlocked=false;}
 else if(strncmp(line,"CLEAR,REAR,",11)==0){rearBlocked=false;}
 else if(strncmp(line,"SENSOR_FAULT,",13)==0){sensorFault=true;state=NavState::SENSOR_FAULT;}
 if(frontBlocked||rearBlocked||sensorFault)sendStop();
}
void readLines(Stream &s,char *buffer,size_t &used,size_t cap,void (*fn)(const char*)){
 while(s.available()){
  char c=s.read();
  if(c=='\n'){buffer[used]=0;if(used)fn(buffer);used=0;}
  else if(c!='\r'){if(used+1<cap)buffer[used++]=c;else used=0;}
 }
}
// BU04 USB data packet layout is observational/experimental.
// 101-byte packets, CmdM:4[ header; ID @11, range mm @17.
// Do NOT assume these bytes are emitted on the module's TTL UART.
uint8_t packet[101];size_t packetUsed=0;
const uint8_t magic[7]={'C','m','d','M',':','4','['};
void readUwb(){
 while(uwb.available()){
  uint8_t b=uwb.read();
  if(packetUsed<7){
   if(b==magic[packetUsed])packet[packetUsed++]=b;
   else packetUsed=(b==magic[0])?1:0;
   if(packetUsed==1)packet[0]=magic[0];
   continue;
  }
  packet[packetUsed++]=b;
  if(packetUsed==101){
   uint32_t id=(uint32_t)packet[11]|((uint32_t)packet[12]<<8)|((uint32_t)packet[13]<<16)|((uint32_t)packet[14]<<24);
   uint16_t mm=(uint16_t)packet[17]|((uint16_t)packet[18]<<8);
   if(id<4 && mm>100 && mm<60000){anchors[id].range=mm;anchors[id].stamp=millis();anchors[id].valid=true;uwbPackets++;}
   else uwbBad++;
   packetUsed=0;
  }
 }
}
void handleConsole(const char *line){
 if(strcmp(line,"HELP")==0)Serial.println("STATUS | ANCHOR,id,x_mm,y_mm | RANGE,id,mm | HEADING,degrees | LEARN | POINT | END | CLEAR_MAP | STOP");
 else if(strcmp(line,"STATUS")==0){
  Serial.printf("state=%d pose=%d antenna=(%.0f,%.0f) center=(%.0f,%.0f) rms=%.0f boundary=%d obstacles=%d uwb=%lu bad=%lu\n",
   (int)state,pose.valid,pose.antenna.x,pose.antenna.y,pose.center.x,pose.center.y,pose.residual,boundaryCount,obstacleCount,(unsigned long)uwbPackets,(unsigned long)uwbBad);
 }
 else if(strcmp(line,"STOP")==0){state=NavState::SAFE_STOP;sendStop();}
 else if(strcmp(line,"LEARN")==0){state=NavState::LEARN_BOUNDARY;boundaryCount=0;Serial.println("Boundary recording armed; use RC only");}
 else if(strcmp(line,"POINT")==0){
  if(state==NavState::LEARN_BOUNDARY && pose.valid && boundaryCount<MAX_BOUNDARY){boundary[boundaryCount++]=pose.center;Serial.printf("Point %d saved\n",boundaryCount);}
  else Serial.println("POINT rejected: need LEARN and valid XY+heading");
 }
 else if(strcmp(line,"END")==0){if(state==NavState::LEARN_BOUNDARY && boundaryCount>=3){state=NavState::READY;Serial.println("Boundary held in RAM; NOT saved to SSD");}}
 else if(strcmp(line,"CLEAR_MAP")==0){boundaryCount=0;state=NavState::SAFE_STOP;}
 else {
  int id;float x,y;
  if(sscanf(line,"ANCHOR,%d,%f,%f",&id,&x,&y)==3 && id>=0 && id<4){
   anchors[id].pos={x,y};anchors[id].valid=false;
   static uint8_t mask=0;mask|=(1<<id);anchorsConfigured=(mask==15);
   Serial.printf("Anchor %d set; all configured=%d\n",id,anchorsConfigured);
  }else if(sscanf(line,"RANGE,%d,%f",&id,&x)==2 && id>=0 && id<4 && x>100 && x<60000){
   anchors[id].range=x;anchors[id].stamp=millis();anchors[id].valid=true;
  }else if(sscanf(line,"HEADING,%f",&x)==1 && isfinite(x)){
   pose.heading=x*PI/180.0f;headingConfigured=true;
   Serial.println("Manual heading set; TEST ONLY, no live IMU");
  }
 }
}
void setup(){
 Serial.begin(115200);cart.begin(Config::BAUD,SERIAL_8N1,Config::CART_RX,Config::CART_TX);
 uwb.begin(Config::BAUD,SERIAL_8N1,Config::UWB_RX,Config::UWB_TX);
 delay(300);Serial.println("Pops S3 Navigation v0.1 SAFE STOP - motors disabled");
 sendStop();
}
void loop(){
 readLines(cart,cartLine,cartLen,sizeof(cartLine),handleCartLine);
 readLines(Serial,usbLine,usbLen,sizeof(usbLine),handleConsole);
 readUwb();updatePose();
 uint32_t now=millis();
 if(now-lastStop>=Config::STOP_PERIOD_MS){lastStop=now;sendStop();}
 if(now-lastPrint>=2000){lastPrint=now;Serial.printf("S3 nav: pose=%d anchors=%d boundary=%d OBS=%d fault=%d\n",pose.valid,anchorsConfigured,boundaryCount,obstacleCount,sensorFault);}
 delay(2);
}
