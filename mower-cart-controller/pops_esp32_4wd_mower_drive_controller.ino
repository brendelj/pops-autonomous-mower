/*
==========================================================================================
 POPS' AUTONOMOUS MOWER - MOWER CART DRIVE CONTROLLER
==========================================================================================
 BOARD / HARDWARE
 -----------------------------------------------------------------------------------------
 Maker / kit: AITRIP
 Product:     ESP-WROOM-32 ESP32 / ESP-32S Type-C USB Development Board
 USB-UART:    CH340C
 Module:      ESP-WROOM-32
 Board type:  30-pin ESP32 development board (15 pins per side)
 USB:         Type-C

 IMPORTANT:
 This diagram matches the actual 30-pin board shown with the AITRIP expansion board.
 Viewed from ABOVE, component side up, with the Type-C connector at the BOTTOM.
 EVERY physical header pin is shown, whether used by this sketch or not.

                                  ANTENNA
                            .-----------------.
                            |   ESP-WROOM-32  |
                            |                 |
 UNUSED                EN  | o             o | D23 / GPIO23  <--- RC/AUTO SWITCH
 UNUSED        VP / GPIO36 | o             o | D22 / GPIO22       UNUSED
 UNUSED        VN / GPIO39 | o             o | TX0 / GPIO1        USB SERIAL TX
 FlySky CH1 STEERING --->34| o             o | RX0 / GPIO3        USB SERIAL RX
 FlySky CH2 THROTTLE --->35| o             o | D21 / GPIO21       UNUSED
 REAR LEFT RPWM       <---32| o             o | D19 / GPIO19  <--- REAR RIGHT LPWM
 REAR LEFT LPWM       <---33| o             o | D18 / GPIO18  <--- REAR RIGHT RPWM
 FRONT LEFT RPWM      <---25| o             o | D5  / GPIO5        UNUSED
 FRONT LEFT LPWM      <---26| o             o | D17 / GPIO17  ---> AUTO ESP32 RX (TX2)
 FRONT RIGHT RPWM     <---27| o             o | D16 / GPIO16  <--- AUTO ESP32 TX (RX2)
 FRONT RIGHT LPWM     <---14| o             o | D4  / GPIO4        UNUSED
 UNUSED / STRAP          12| o             o | D2  / GPIO2        UNUSED / STRAP
 BTS7960 ENABLE       <---13| o             o | D15 / GPIO15       UNUSED / STRAP
 COMMON GROUND          GND| o             o | GND                COMMON GROUND
 +5V BUCK ------------> VIN| o             o | 3V3                UNUSED
                            |                 |
                            |     TYPE-C      |
                            '-------| |-------'

 PHYSICAL HEADER ORDER (TOP -> BOTTOM)
 -----------------------------------------------------------------------------------------
 LEFT SIDE                               RIGHT SIDE
 EN                                      D23 / GPIO23
 VP / GPIO36                             D22 / GPIO22
 VN / GPIO39                             TX0 / GPIO1
 D34 / GPIO34                            RX0 / GPIO3
 D35 / GPIO35                            D21 / GPIO21
 D32 / GPIO32                            D19 / GPIO19
 D33 / GPIO33                            D18 / GPIO18
 D25 / GPIO25                            D5  / GPIO5
 D26 / GPIO26                            D17 / GPIO17
 D27 / GPIO27                            D16 / GPIO16
 D14 / GPIO14                            D4  / GPIO4
 D12 / GPIO12                            D2  / GPIO2
 D13 / GPIO13                            D15 / GPIO15
 GND                                     GND
 VIN                                     3V3

==========================================================================================
 LOCKED GPIO ASSIGNMENTS - SOURCE OF TRUTH
==========================================================================================
 RC INPUT
   GPIO34  FlySky CH1 steering
   GPIO35  FlySky CH2 throttle

 PHYSICAL CONTROL SOURCE SWITCH
   GPIO23  RC/AUTO selector using INPUT_PULLUP
           Switch OPEN          = RC MODE
           Switch CLOSED to GND = AUTONOMOUS MODE

 AUTONOMOUS ESP32 UART
   GPIO16  RX2 <- Autonomous ESP32 TX
   GPIO17  TX2 -> Autonomous ESP32 RX (optional status/debug)

 FOUR BTS7960 MOTOR CONTROLLERS
   FRONT LEFT    GPIO25 RPWM    GPIO26 LPWM
   FRONT RIGHT   GPIO27 RPWM    GPIO14 LPWM
   REAR LEFT     GPIO32 RPWM    GPIO33 LPWM
   REAR RIGHT    GPIO18 RPWM    GPIO19 LPWM

   GPIO13 -> R_EN and L_EN on ALL FOUR BTS7960 boards

 POWER
   12V battery -> appropriately fused motor distribution -> BTS7960 motor power
   12V battery -> regulated 5V buck -> ESP32 VIN
   All controller/receiver logic supplies must be appropriate for their hardware.
   ALL system grounds must share a common reference.

 WHEEL CALIBRATION
   Front wheel diameter = 11.50 inches
   Rear wheel diameter  = 12.25 inches
   Front wheel scale    = 12.25 / 11.50 = 1.065217

   FL_TRIM, FR_TRIM, RL_TRIM and RR_TRIM remain independently adjustable.

 AUTONOMOUS SERIAL COMMANDS
   DRV,<throttle>,<steering>
   throttle and steering range: -1000 through +1000
   STOP

 FAILSAFE
   RC signal timeout stops all motors.
   Autonomous command timeout stops all motors.
   Changing RC/AUTO mode forces a temporary stop.
==========================================================================================
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
