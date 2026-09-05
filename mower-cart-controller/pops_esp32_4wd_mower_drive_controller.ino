/*
=====================================================================
 POPS' ESP32 4-WHEEL MOWER DRIVE CONTROLLER
=====================================================================
 ESP32 PINOUT - SOURCE OF TRUTH
 GPIO34 FlySky CH1 steering
 GPIO35 FlySky CH2 throttle
 GPIO23 RC/AUTO switch (LOW=AUTO)
 GPIO16 RX2 <- Autonomous ESP32 TX
 GPIO17 TX2 -> Autonomous ESP32 RX
 GPIO25 FL RPWM   GPIO26 FL LPWM
 GPIO27 FR RPWM   GPIO14 FR LPWM
 GPIO32 RL RPWM   GPIO33 RL LPWM
 GPIO18 RR RPWM   GPIO19 RR LPWM
 GPIO13 shared BTS7960 R_EN/L_EN

 Front wheel: 11.50"
 Rear wheel:  12.25"
 Front scale: 12.25/11.50 = 1.065217

 AUTO serial:
 DRV,<throttle>,<steering>  (-1000..1000)
 STOP
=====================================================================
*/
#include <Arduino.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

const uint8_t PIN_RC_STEERING=34, PIN_RC_THROTTLE=35, PIN_AUTO_SWITCH=23;
const uint8_t PIN_AUTO_RX=16, PIN_AUTO_TX=17;
const uint8_t PIN_FL_RPWM=25, PIN_FL_LPWM=26;
const uint8_t PIN_FR_RPWM=27, PIN_FR_LPWM=14;
const uint8_t PIN_RL_RPWM=32, PIN_RL_LPWM=33;
const uint8_t PIN_RR_RPWM=18, PIN_RR_LPWM=19;
const uint8_t PIN_MOTOR_ENABLE=13;

const float FRONT_WHEEL_DIAMETER=11.50f, REAR_WHEEL_DIAMETER=12.25f;
const float FRONT_WHEEL_SCALE=REAR_WHEEL_DIAMETER/FRONT_WHEEL_DIAMETER;
float FL_TRIM=1.000f, FR_TRIM=1.000f, RL_TRIM=1.000f, RR_TRIM=1.000f;
bool FL_REVERSED=false, FR_REVERSED=true, RL_REVERSED=false, RR_REVERSED=true;

int RC_STEER_LEFT_US=1000, RC_STEER_CENTER_US=1500, RC_STEER_RIGHT_US=2000;
int RC_THROTTLE_REV_US=1000, RC_THROTTLE_NEUTRAL_US=1500, RC_THROTTLE_FWD_US=2000;
const int RC_STEER_DEADBAND_US=25, RC_THROTTLE_DEADBAND_US=35;
const uint32_t RC_TIMEOUT_MS=150, AUTO_TIMEOUT_MS=300, MODE_CHANGE_STOP_MS=300;
const uint32_t PWM_FREQ=16000; const uint8_t PWM_BITS=10;
const uint16_t PWM_MAX=(1<<PWM_BITS)-1;

volatile uint32_t steeringRiseUs=0, throttleRiseUs=0;
volatile uint16_t steeringPulseUs=1500, throttlePulseUs=1500;
volatile uint32_t steeringLastPulseMs=0, throttleLastPulseMs=0;
float autoThrottle=0.0f, autoSteering=0.0f;
uint32_t lastAutoCommandMs=0, modeChangeMs=0;
String autoBuffer; bool previousAutoMode=false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void configurePwmPin(uint8_t pin){ ledcAttach(pin,PWM_FREQ,PWM_BITS); }
void writePwm(uint8_t pin,uint16_t duty){ ledcWrite(pin,duty); }
#else
struct PwmMapEntry{uint8_t pin;uint8_t channel;}; PwmMapEntry pwmMap[8];
uint8_t pwmMapCount=0,pwmChannelCounter=0;
void configurePwmPin(uint8_t pin){uint8_t ch=pwmChannelCounter++;ledcSetup(ch,PWM_FREQ,PWM_BITS);ledcAttachPin(pin,ch);pwmMap[pwmMapCount++]={pin,ch};}
void writePwm(uint8_t pin,uint16_t duty){for(uint8_t i=0;i<pwmMapCount;i++)if(pwmMap[i].pin==pin){ledcWrite(pwmMap[i].channel,duty);return;}}
#endif

void IRAM_ATTR steeringISR(){if(digitalRead(PIN_RC_STEERING))steeringRiseUs=micros();else{uint32_t w=micros()-steeringRiseUs;if(w>=800&&w<=2200){steeringPulseUs=w;steeringLastPulseMs=millis();}}}
void IRAM_ATTR throttleISR(){if(digitalRead(PIN_RC_THROTTLE))throttleRiseUs=micros();else{uint32_t w=micros()-throttleRiseUs;if(w>=800&&w<=2200){throttlePulseUs=w;throttleLastPulseMs=millis();}}}

float convertCenteredRc(int p,int mn,int ctr,int mx,int db){
 if(abs(p-ctr)<=db)return 0.0f;
 float v=(p>ctr)?(float)(p-ctr)/(mx-ctr):(float)(p-ctr)/(ctr-mn);
 return constrain(v,-1.0f,1.0f);
}
void stopMotor(uint8_t r,uint8_t l){writePwm(r,0);writePwm(l,0);}
void setMotor(uint8_t r,uint8_t l,float c,bool rev){
 c=constrain(c,-1.0f,1.0f); if(rev)c=-c; uint16_t d=(uint16_t)(fabs(c)*PWM_MAX);
 if(c>0.001f){writePwm(l,0);writePwm(r,d);}else if(c<-0.001f){writePwm(r,0);writePwm(l,d);}else stopMotor(r,l);
}
void stopAllMotors(){stopMotor(PIN_FL_RPWM,PIN_FL_LPWM);stopMotor(PIN_FR_RPWM,PIN_FR_LPWM);stopMotor(PIN_RL_RPWM,PIN_RL_LPWM);stopMotor(PIN_RR_RPWM,PIN_RR_LPWM);}
void normalizeWheelCommands(float&fl,float&fr,float&rl,float&rr){
 float m=max(max(fabs(fl),fabs(fr)),max(fabs(rl),fabs(rr)));
 if(m>1.0f){fl/=m;fr/=m;rl/=m;rr/=m;}
}
void driveMower(float throttle,float steering){
 float left=throttle+steering,right=throttle-steering,m=max(fabs(left),fabs(right));
 if(m>1.0f){left/=m;right/=m;}
 float fl=left*FRONT_WHEEL_SCALE*FL_TRIM,fr=right*FRONT_WHEEL_SCALE*FR_TRIM;
 float rl=left*RL_TRIM,rr=right*RR_TRIM; normalizeWheelCommands(fl,fr,rl,rr);
 setMotor(PIN_FL_RPWM,PIN_FL_LPWM,fl,FL_REVERSED);setMotor(PIN_FR_RPWM,PIN_FR_LPWM,fr,FR_REVERSED);
 setMotor(PIN_RL_RPWM,PIN_RL_LPWM,rl,RL_REVERSED);setMotor(PIN_RR_RPWM,PIN_RR_LPWM,rr,RR_REVERSED);
}
void processAutoCommand(String s){
 s.trim(); if(s.equalsIgnoreCase("STOP")){autoThrottle=autoSteering=0;lastAutoCommandMs=millis();return;}
 if(!s.startsWith("DRV,"))return; int c1=s.indexOf(','),c2=s.indexOf(',',c1+1);if(c1<0||c2<0)return;
 int t=constrain(s.substring(c1+1,c2).toInt(),-1000,1000),st=constrain(s.substring(c2+1).toInt(),-1000,1000);
 autoThrottle=t/1000.0f;autoSteering=st/1000.0f;lastAutoCommandMs=millis();
}
void readAutoSerial(){while(Serial2.available()){char c=Serial2.read();if(c=='\n'){processAutoCommand(autoBuffer);autoBuffer="";}else if(c!='\r'){if(autoBuffer.length()<80)autoBuffer+=c;else autoBuffer="";}}}

void setup(){
 Serial.begin(115200);Serial2.begin(115200,SERIAL_8N1,PIN_AUTO_RX,PIN_AUTO_TX);
 pinMode(PIN_RC_STEERING,INPUT);pinMode(PIN_RC_THROTTLE,INPUT);pinMode(PIN_AUTO_SWITCH,INPUT_PULLUP);
 pinMode(PIN_MOTOR_ENABLE,OUTPUT);digitalWrite(PIN_MOTOR_ENABLE,LOW);
 const uint8_t pins[]={PIN_FL_RPWM,PIN_FL_LPWM,PIN_FR_RPWM,PIN_FR_LPWM,PIN_RL_RPWM,PIN_RL_LPWM,PIN_RR_RPWM,PIN_RR_LPWM};
 for(uint8_t p:pins)configurePwmPin(p);stopAllMotors();
 attachInterrupt(digitalPinToInterrupt(PIN_RC_STEERING),steeringISR,CHANGE);
 attachInterrupt(digitalPinToInterrupt(PIN_RC_THROTTLE),throttleISR,CHANGE);
 delay(1000);digitalWrite(PIN_MOTOR_ENABLE,HIGH);
 previousAutoMode=digitalRead(PIN_AUTO_SWITCH)==LOW;modeChangeMs=millis();
}
void loop(){
 readAutoSerial();bool autoMode=digitalRead(PIN_AUTO_SWITCH)==LOW;
 if(autoMode!=previousAutoMode){stopAllMotors();modeChangeMs=millis();previousAutoMode=autoMode;}
 if(millis()-modeChangeMs<MODE_CHANGE_STOP_MS){stopAllMotors();delay(5);return;}
 if(autoMode){
   if(millis()-lastAutoCommandMs>AUTO_TIMEOUT_MS){stopAllMotors();delay(5);return;}
   driveMower(autoThrottle,autoSteering);
 }else{
   uint32_t now=millis();if(now-steeringLastPulseMs>=RC_TIMEOUT_MS||now-throttleLastPulseMs>=RC_TIMEOUT_MS){stopAllMotors();delay(5);return;}
   uint16_t sp,tp;noInterrupts();sp=steeringPulseUs;tp=throttlePulseUs;interrupts();
   float s=convertCenteredRc(sp,RC_STEER_LEFT_US,RC_STEER_CENTER_US,RC_STEER_RIGHT_US,RC_STEER_DEADBAND_US);
   float t=convertCenteredRc(tp,RC_THROTTLE_REV_US,RC_THROTTLE_NEUTRAL_US,RC_THROTTLE_FWD_US,RC_THROTTLE_DEADBAND_US);
   driveMower(t,s);
 }delay(5);
}
