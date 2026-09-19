/*
==========================================================================================
 POPS' AUTONOMOUS MOWER - MOWER CART DRIVE CONTROLLER                     (revision 3)
==========================================================================================
 REVISION 3 REVIEW / CHANGES
 -----------------------------------------------------------------------------------------
 REVISION 3 ADDS TO THE REVISION 2 SAFETY WORK
   1. Autonomous DRV values outside -1000..+1000 are now REJECTED instead of silently
      clamped. A malformed/corrupted command therefore cannot become a valid full-speed
      command merely because it parsed as a large integer.
   2. Sonar documentation corrected: a JSN-SR04T timeout is inherently ambiguous. It can
      mean open space, a weak/angled target, a target outside range, or (depending on the
      module/target) a target inside the near blind zone. Firmware cannot prove which one.
      The existing post-arming "no echo = open space" policy is retained for outdoor use,
      but it is NOT a complete collision-safety mechanism. A hardware e-stop remains
      required; a bumper/near-field sensor is recommended if close-range coverage matters.
   3. Comments now explicitly warn that the 12-inch stop threshold must be validated at
      the mower's actual maximum speed on grass; MAX_THROTTLE should be reduced until the
      measured stopping distance has adequate margin.

 REVISION 2 CHANGES
 -----------------------------------------------------------------------------------------
 FIRMWARE
   1. RC timeout race fixed: pulse widths AND their timestamps are snapshotted together
      with interrupts off, so a pulse landing mid-check can no longer cause a false stop.
   2. Ultrasonic logic rewritten around four outcomes:
        VALID      echo between 20 cm and 600 cm
        TOO CLOSE  echo under 20 cm (blind zone) -> that direction is BLOCKED IMMEDIATELY
        NO ECHO    nothing within ~5 m. Once a sensor has been ARMED by one valid echo
                   since power-up, no echo means "clear". Until armed it stays BLOCKED.
                   Set SONAR_NO_ECHO_IS_CLEAR=false to restore the strict first-version
                   behavior (no echo = fault) - expect lockouts in open ground.
        STUCK      ECHO already HIGH before a trigger -> counts toward a sensor fault
      To arm a sensor after power-up, put a hand or board 50-100 cm in front of it.
   3. Serial2 reports fixed: SENSOR_FAULT never carries a distance, and a fault-caused
      block no longer emits OBS with a stale distance (could map a phantom obstacle).
   4. Serial2 parser is strict: numbers validated, extra fields rejected, optional XOR
      checksum, and an oversized line is discarded up to its newline.
   5. Throttle/steering slew limiting. Obstacle-interlock cuts are never ramped.
   6. RC sticks must return to neutral after boot, mode change, or RC signal loss before
      the cart will move. AUTO ignores any DRV received before the mode switch.
   7. RC/AUTO selector is debounced.
   8. MAX_THROTTLE speed cap, and a 1 second task watchdog.
   9. 1 Hz status line on USB serial (115200) for bench testing.

 HARDWARE ADDITIONS (not in the first version - please add)
   - FlySky CH1/CH2 into GPIO34/35: if the receiver runs from 5V its signal is ~5V.
     Use the same divider as the JSN echo lines: signal -1K- GPIO, GPIO -2K- GND.
   - 10K pull-down from GPIO13 to GND so the BTS7960 enables stay off during reset/boot.
   - Hardware e-stop / contactor in the +12V motor bus that does not depend on firmware.
   - Set the receiver failsafe (throttle neutral) and TEST it with the transmitter off.
   - Bulk capacitor (470 uF or more) near the 5V buck output.

==========================================================================================
 BOARD / HARDWARE
 -----------------------------------------------------------------------------------------
 Maker / kit: AITRIP
 Product:     ESP-WROOM-32 ESP32 / ESP-32S Type-C USB Development Board
 USB-UART:    CH340C
 Module:      ESP-WROOM-32
 Board type:  30-pin ESP32 development board (15 pins per side)
 USB:         Type-C

 Viewed from ABOVE, component side up, Type-C connector at the BOTTOM.
 EVERY physical header pin is shown.

                                  ANTENNA
                            .-----------------.
                            |   ESP-WROOM-32  |
                            |                 |
 UNUSED                EN  | o             o | D23 / GPIO23  <--- RC/AUTO SWITCH
 UNUSED        VP / GPIO36 | o             o | D22 / GPIO22  <--- FRONT JSN ECHO *
 UNUSED        VN / GPIO39 | o             o | TX0 / GPIO1        USB SERIAL TX
 FlySky CH1 STEERING --->34| o             o | RX0 / GPIO3        USB SERIAL RX
 FlySky CH2 THROTTLE --->35| o             o | D21 / GPIO21  ---> FRONT JSN TRIG
 REAR LEFT RPWM       <---32| o             o | D19 / GPIO19  ---> REAR RIGHT LPWM
 REAR LEFT LPWM       <---33| o             o | D18 / GPIO18  ---> REAR RIGHT RPWM
 FRONT LEFT RPWM      <---25| o             o | D5  / GPIO5   <--- REAR JSN ECHO *
 FRONT LEFT LPWM      <---26| o             o | D17 / GPIO17  ---> AUTO ESP32 RX (TX2)
 FRONT RIGHT RPWM     <---27| o             o | D16 / GPIO16  <--- AUTO ESP32 TX (RX2)
 FRONT RIGHT LPWM     <---14| o             o | D4  / GPIO4   ---> REAR JSN TRIG
 UNUSED / STRAP          12| o             o | D2  / GPIO2        UNUSED / STRAP
 BTS7960 ENABLE       <---13| o             o | D15 / GPIO15       UNUSED / STRAP
 COMMON GROUND          GND| o             o | GND                COMMON GROUND
 +5V BUCK ------------> VIN| o             o | 3V3                UNUSED
                            |                 |
                            |     TYPE-C      |
                            '-------| |-------'

 * JSN-SR04T ECHO is 5V. DO NOT connect ECHO directly to ESP32.
   Use a 5V -> 3.3V divider/level shifter on GPIO22 and GPIO5.

 PHYSICAL HEADER ORDER (TOP -> BOTTOM)
 -----------------------------------------------------------------------------------------
 LEFT SIDE                               RIGHT SIDE
 EN        UNUSED                        GPIO23   RC/AUTO SWITCH
 GPIO36    UNUSED                        GPIO22   FRONT JSN-SR04T ECHO *
 GPIO39    UNUSED                        GPIO1    USB SERIAL TX
 GPIO34    FLYSKY CH1 STEERING           GPIO3    USB SERIAL RX
 GPIO35    FLYSKY CH2 THROTTLE           GPIO21   FRONT JSN-SR04T TRIG
 GPIO32    REAR LEFT RPWM                GPIO19   REAR RIGHT LPWM
 GPIO33    REAR LEFT LPWM                GPIO18   REAR RIGHT RPWM
 GPIO25    FRONT LEFT RPWM               GPIO5    REAR JSN-SR04T ECHO *
 GPIO26    FRONT LEFT LPWM               GPIO17   AUTO ESP32 RX
 GPIO27    FRONT RIGHT RPWM              GPIO16   AUTO ESP32 TX
 GPIO14    FRONT RIGHT LPWM              GPIO4    REAR JSN-SR04T TRIG
 GPIO12    UNUSED / STRAP                GPIO2    UNUSED / STRAP
 GPIO13    ALL BTS7960 ENABLES           GPIO15   UNUSED / STRAP
 GND       COMMON GROUND                 GND      COMMON GROUND
 VIN       +5V FROM BUCK                 3V3      UNUSED

==========================================================================================
 LOCKED GPIO ASSIGNMENTS - SOURCE OF TRUTH
==========================================================================================
 RC INPUT
   GPIO34 <- FlySky CH1 steering
   GPIO35 <- FlySky CH2 throttle

 RC / AUTONOMOUS SELECTOR
   GPIO23 <- selector switch using INPUT_PULLUP
             OPEN          = RC MODE
             CLOSED -> GND = AUTONOMOUS MODE

 AUTONOMOUS ESP32 UART
   GPIO16 RX2 <- Autonomous ESP32 TX
   GPIO17 TX2 -> Autonomous ESP32 RX

 FRONT JSN-SR04T
   GPIO21 -> TRIG
   GPIO22 <- ECHO through 5V -> 3.3V level conversion
   VCC    -> regulated +5V
   GND    -> common ground

 REAR JSN-SR04T
   GPIO4  -> TRIG
   GPIO5  <- ECHO through 5V -> 3.3V level conversion
   VCC    -> regulated +5V
   GND    -> common ground

 FOUR BTS7960 MOTOR CONTROLLERS
   FRONT LEFT    GPIO25 RPWM    GPIO26 LPWM
   FRONT RIGHT   GPIO27 RPWM    GPIO14 LPWM
   REAR LEFT     GPIO32 RPWM    GPIO33 LPWM
   REAR RIGHT    GPIO18 RPWM    GPIO19 LPWM

   GPIO13 -> R_EN and L_EN on ALL FOUR BTS7960 boards

 BTS7960 PHYSICAL CONNECTIONS - MODULE ORIENTATION FROM POPS' REFERENCE PHOTO
   Pin 1  RPWM  <- motor-specific RPWM GPIO
   Pin 2  LPWM  <- motor-specific LPWM GPIO
   Pin 3  R_EN  <- GPIO13 shared enable
   Pin 4  L_EN  <- GPIO13 shared enable
   Pin 5  R_IS  -- not used
   Pin 6  L_IS  -- not used
   Pin 7  VCC   <- regulated +5V logic
   Pin 8  GND   <- common ground
   Pin 9  MOTOR -
   Pin 10 MOTOR +
   Pin 11 VM+   <- fused/switched +12V motor bus
   Pin 12 VM-   <- battery negative/common ground

 JSN-SR04T ECHO LEVEL CONVERSION - EACH SENSOR
   JSN ECHO (5V) -- 1K --+--> ESP32 ECHO GPIO
                          |
                          2K
                          |
                         GND
   Front midpoint -> GPIO22
   Rear midpoint  -> GPIO5

 OBSTACLE SAFETY
   STOP/BLOCK distance = 12.0 in / 30.48 cm
   CLEAR distance      = 15.0 in / 38.10 cm (hysteresis)
   Front obstruction blocks positive/forward throttle.
   Rear obstruction blocks negative/reverse throttle.
   Steering remains available so RC operator/autonomous controller can turn away.
   Motion in the opposite direction remains available.
   Sensors start fail-safe BLOCKED until a valid reading proves clearance (arming).
   A returned echo measured inside the configured blind-zone limit blocks immediately.
   IMPORTANT: a NO-ECHO timeout cannot distinguish open space from every possible near-field
   failure/target condition. Do not treat these two sonars as the mower's only collision
   protection. A physical bumper/near-field backup sensor is recommended.
   A blocked direction is only cleared by a valid reading at or beyond the CLEAR distance;
   "no echo" never clears a blocked direction.
   Steering is not covered: the sensors watch straight ahead/behind only, so a pivot can
   still swing the ends of the cart into something.

 AUTONOMOUS OBSTACLE REPORTS ON Serial2
   OBS,FRONT,<millimeters>
   OBS,REAR,<millimeters>
   CLEAR,FRONT,<millimeters>
   CLEAR,REAR,<millimeters>
   SENSOR_FAULT,FRONT
   SENSOR_FAULT,REAR

   The cart controller also retains a small recent detection history in RAM.
   A geometric/global obstacle outline requires mower position/heading, so the
   autonomous ESP32 must combine these reports with its navigation pose and store
   the mapped obstacle boundary there.

 POWER
   12V battery -> fuse -> master switch -> +12V motor distribution -> BTS7960 VM+
   12V battery -> regulated 5V buck -> ESP32 VIN, JSN-SR04T VCC, BTS7960 VCC
   Verify FlySky receiver supply voltage before connecting its VCC.
   ALL controller, receiver, sensor, motor-driver and autonomous ESP32 grounds
   must share a common reference.

 WHEEL CALIBRATION
   Front wheel diameter = 11.50 inches
   Rear wheel diameter  = 12.25 inches
   Front wheel scale    = 12.25 / 11.50 = 1.065217

 AUTONOMOUS SERIAL COMMANDS (from the autonomous ESP32, 115200 8N1, newline terminated)
   DRV,<throttle>,<steering>[*HH]   (-1000 through +1000)
   STOP[*HH]
   *HH is an optional two-digit hex XOR of every character before the '*'.
   Examples:  DRV,500,-200*6A     STOP*18     DRV,0,0*40
   Sender (autonomous ESP32):
      String body="DRV,500,-200"; uint8_t c=0; for(char ch:body) c^=(uint8_t)ch;
      char hh[4]; snprintf(hh,sizeof(hh),"%02X",c); Serial2.println(body+"*"+hh);
   Set AUTO_REQUIRE_CHECKSUM=true below to reject DRV lines without a checksum.
   STOP is always accepted, with or without a checksum, because it is the safe direction.
   The autonomous controller must keep sending DRV at least every 300 ms.

 FAILSAFE
   RC signal timeout stops all motors and requires the sticks to return to neutral.
   Autonomous command timeout stops all motors.
   Changing RC/AUTO mode forces a temporary stop and discards old autonomous commands.
   Ultrasonic direction interlock is applied AFTER either RC or autonomous command,
   and after the ramp, so a cut is immediate.
   A hung main loop resets the ESP32 after WDT_TIMEOUT_S (motor pins go inactive).
==========================================================================================
*/
#include <Arduino.h>
#include <esp_task_wdt.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define CORE_V3 1
#else
#define CORE_V3 0
#endif

const uint8_t PIN_RC_STEERING=34, PIN_RC_THROTTLE=35, PIN_AUTO_SWITCH=23;
const uint8_t PIN_AUTO_RX=16, PIN_AUTO_TX=17;
const uint8_t PIN_FL_RPWM=25, PIN_FL_LPWM=26;
const uint8_t PIN_FR_RPWM=27, PIN_FR_LPWM=14;
const uint8_t PIN_RL_RPWM=32, PIN_RL_LPWM=33;
const uint8_t PIN_RR_RPWM=18, PIN_RR_LPWM=19;
const uint8_t PIN_MOTOR_ENABLE=13;
const uint8_t PIN_FRONT_TRIG=21, PIN_FRONT_ECHO=22;
const uint8_t PIN_REAR_TRIG=4, PIN_REAR_ECHO=5;

const float FRONT_WHEEL_DIAMETER=11.50f, REAR_WHEEL_DIAMETER=12.25f;
const float FRONT_WHEEL_SCALE=REAR_WHEEL_DIAMETER/FRONT_WHEEL_DIAMETER;
float FL_TRIM=1.000f, FR_TRIM=1.000f, RL_TRIM=1.000f, RR_TRIM=1.000f;
bool FL_REVERSED=false, FR_REVERSED=true, RL_REVERSED=false, RR_REVERSED=true;

int RC_STEER_LEFT_US=1000, RC_STEER_CENTER_US=1500, RC_STEER_RIGHT_US=2000;
int RC_THROTTLE_REV_US=1000, RC_THROTTLE_NEUTRAL_US=1500, RC_THROTTLE_FWD_US=2000;
const int RC_STEER_DEADBAND_US=25, RC_THROTTLE_DEADBAND_US=35;

const uint32_t RC_TIMEOUT_MS=150, AUTO_TIMEOUT_MS=300, MODE_CHANGE_STOP_MS=300;
const uint32_t RC_NEUTRAL_HOLD_MS=300;
const uint32_t SWITCH_DEBOUNCE_MS=50;
const uint32_t STATUS_INTERVAL_MS=1000;
const uint32_t WDT_TIMEOUT_S=1;
const uint32_t PWM_FREQ=16000; const uint8_t PWM_BITS=10;
const uint16_t PWM_MAX=(1<<PWM_BITS)-1;

const float MAX_THROTTLE=1.0f;
const float RAMP_ACCEL_PER_SEC=2.0f;
const float RAMP_DECEL_PER_SEC=6.0f;

const bool AUTO_REQUIRE_CHECKSUM=false;

const float OBSTACLE_STOP_CM=30.48f;
const float OBSTACLE_CLEAR_CM=38.10f;
const float SONAR_MIN_VALID_CM=20.0f;
const float SONAR_MAX_VALID_CM=600.0f;
const uint32_t SONAR_SAMPLE_INTERVAL_MS=70;
const uint32_t SONAR_ECHO_TIMEOUT_US=30000;
const uint8_t SONAR_FAULT_LIMIT=3;
const bool SONAR_NO_ECHO_IS_CLEAR=true;

enum SonarResult : uint8_t { SONAR_OK, SONAR_TOO_CLOSE, SONAR_NO_ECHO, SONAR_STUCK };

struct SonarState {
  uint8_t trigPin; uint8_t echoPin; const char* name; float distanceCm;
  bool blocked; bool fault; bool armed; uint8_t invalidCount; uint32_t lastRememberMs;
};

SonarState frontSonar={PIN_FRONT_TRIG,PIN_FRONT_ECHO,"FRONT",-1.0f,true,false,false,0,0};
SonarState rearSonar ={PIN_REAR_TRIG,PIN_REAR_ECHO,"REAR",-1.0f,true,false,false,0,0};
uint32_t lastSonarSampleMs=0; bool sampleFrontNext=true;

struct ObstacleObservation { uint32_t timeMs; bool front; uint16_t distanceMm; };
const uint8_t OBSTACLE_HISTORY_SIZE=32;
ObstacleObservation obstacleHistory[OBSTACLE_HISTORY_SIZE];
uint8_t obstacleHistoryHead=0, obstacleHistoryCount=0;

volatile uint32_t steeringRiseUs=0, throttleRiseUs=0;
volatile uint16_t steeringPulseUs=1500, throttlePulseUs=1500;
volatile uint32_t steeringLastPulseMs=0, throttleLastPulseMs=0;
float autoThrottle=0.0f, autoSteering=0.0f;
bool autoCommandValid=false;
uint32_t lastAutoCommandMs=0, modeChangeMs=0;
String autoBuffer; bool autoDiscard=false;
bool previousAutoMode=false;
bool switchStable=false, switchCandidate=false; uint32_t switchCandidateMs=0;
bool rcArmed=false, rcNeutralTiming=false; uint32_t rcNeutralStartMs=0;
float rampThrottle=0.0f, rampSteering=0.0f; uint32_t lastRampMs=0;
uint32_t lastStatusMs=0;

#if CORE_V3
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
void haltAll(){stopAllMotors();rampThrottle=0.0f;rampSteering=0.0f;lastRampMs=millis();}

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

void rememberObstacle(bool front,float cm){
 ObstacleObservation &o=obstacleHistory[obstacleHistoryHead]; o.timeMs=millis();o.front=front;
 float mm=cm*10.0f;if(mm<0)mm=0;if(mm>65535)mm=65535;o.distanceMm=(uint16_t)mm;
 obstacleHistoryHead=(obstacleHistoryHead+1)%OBSTACLE_HISTORY_SIZE;
 if(obstacleHistoryCount<OBSTACLE_HISTORY_SIZE)obstacleHistoryCount++;
}
void reportSonarEvent(const char* eventName,const SonarState& s,bool withDistance,float cm){
 Serial2.print(eventName);Serial2.print(',');Serial2.print(s.name);
 if(withDistance){Serial2.print(',');Serial2.print((int)(cm*10.0f));}Serial2.println();
}
SonarResult readJsnSr04t(const SonarState& s,float& cm){
 if(digitalRead(s.echoPin)==HIGH)return SONAR_STUCK;
 digitalWrite(s.trigPin,LOW);delayMicroseconds(3);digitalWrite(s.trigPin,HIGH);delayMicroseconds(12);digitalWrite(s.trigPin,LOW);
 unsigned long echoUs=pulseIn(s.echoPin,HIGH,SONAR_ECHO_TIMEOUT_US);
 if(echoUs==0)return SONAR_NO_ECHO;
 cm=(echoUs*0.0343f)/2.0f;
 if(cm<SONAR_MIN_VALID_CM)return SONAR_TOO_CLOSE;
 if(cm>SONAR_MAX_VALID_CM)return SONAR_NO_ECHO;
 return SONAR_OK;
}
void updateOneSonar(SonarState& s,bool isFront){
 const bool wasBlocked=s.blocked,wasFault=s.fault;float cm=-1.0f;
 const SonarResult r=readJsnSr04t(s,cm);bool haveDistance=false,invalid=false;
 switch(r){
  case SONAR_OK:
   s.invalidCount=0;s.fault=false;s.armed=true;s.distanceCm=cm;haveDistance=true;
   if(s.blocked){if(cm>=OBSTACLE_CLEAR_CM)s.blocked=false;}else if(cm<=OBSTACLE_STOP_CM)s.blocked=true;break;
  case SONAR_TOO_CLOSE:
   s.invalidCount=0;s.fault=false;s.armed=true;s.distanceCm=cm;haveDistance=true;s.blocked=true;break;
  case SONAR_NO_ECHO:
   if(SONAR_NO_ECHO_IS_CLEAR&&s.armed){s.invalidCount=0;s.fault=false;s.distanceCm=-1.0f;}else invalid=true;break;
  case SONAR_STUCK: invalid=true;break;
 }
 if(invalid){if(s.invalidCount<255)s.invalidCount++;if(s.invalidCount>=SONAR_FAULT_LIMIT){s.fault=true;s.blocked=true;}}
 if(!wasFault&&s.fault)reportSonarEvent("SENSOR_FAULT",s,false,0.0f);
 if(wasBlocked!=s.blocked){
  if(s.blocked){if(haveDistance){rememberObstacle(isFront,s.distanceCm);reportSonarEvent("OBS",s,true,s.distanceCm);}}
  else reportSonarEvent("CLEAR",s,true,s.distanceCm);
 }else if(s.blocked&&haveDistance){
  if(wasFault&&!s.fault){rememberObstacle(isFront,s.distanceCm);reportSonarEvent("OBS",s,true,s.distanceCm);s.lastRememberMs=millis();}
  else if(millis()-s.lastRememberMs>=500){rememberObstacle(isFront,s.distanceCm);s.lastRememberMs=millis();}
 }
}
void updateSonars(){
 uint32_t now=millis();if(now-lastSonarSampleMs<SONAR_SAMPLE_INTERVAL_MS)return;lastSonarSampleMs=now;
 if(sampleFrontNext)updateOneSonar(frontSonar,true);else updateOneSonar(rearSonar,false);sampleFrontNext=!sampleFrontNext;
}
bool applyObstacleInterlock(float& throttle,float& steering){
 bool cut=false;if(throttle>0.0f&&frontSonar.blocked){throttle=0.0f;cut=true;}
 if(throttle<0.0f&&rearSonar.blocked){throttle=0.0f;cut=true;}steering=constrain(steering,-1.0f,1.0f);return cut;
}

float slewLimit(float cur,float target,float dt){
 float delta=target-cur;const bool slowingDown=(fabsf(target)<fabsf(cur))&&(cur*target>=0.0f);
 const float maxStep=(slowingDown?RAMP_DECEL_PER_SEC:RAMP_ACCEL_PER_SEC)*dt;
 if(delta>maxStep)delta=maxStep;else if(delta<-maxStep)delta=-maxStep;return cur+delta;
}
void driveWithRamp(float targetThrottle,float targetSteering){
 uint32_t now=millis();float dt=(now-lastRampMs)/1000.0f;lastRampMs=now;if(dt>0.1f)dt=0.1f;
 targetThrottle=constrain(targetThrottle,-1.0f,1.0f)*MAX_THROTTLE;targetSteering=constrain(targetSteering,-1.0f,1.0f);
 rampThrottle=slewLimit(rampThrottle,targetThrottle,dt);rampSteering=slewLimit(rampSteering,targetSteering,dt);
 float t=rampThrottle,s=rampSteering;applyObstacleInterlock(t,s);rampThrottle=t;driveMower(t,s);
}

void autoStopCommand(){autoThrottle=0.0f;autoSteering=0.0f;autoCommandValid=true;lastAutoCommandMs=millis();}
bool parseIntStrict(const String& t,int& out){
 int n=t.length();if(n<=0||n>6)return false;int i=(t[0]=='-'||t[0]=='+')?1:0;if(i>=n)return false;
 for(;i<n;i++)if(t[i]<'0'||t[i]>'9')return false;out=t.toInt();return true;
}
bool stripChecksum(String& s){
 int star=s.lastIndexOf('*');if(star<0)return !AUTO_REQUIRE_CHECKSUM;if((int)s.length()-star!=3)return false;
 uint8_t calc=0;for(int i=0;i<star;i++)calc^=(uint8_t)s[i];String hex=s.substring(star+1);
 char* endp=nullptr;long given=strtol(hex.c_str(),&endp,16);if(endp==hex.c_str()||*endp!='\0')return false;
 if(calc!=(uint8_t)given)return false;s.remove(star);return true;
}
void processAutoCommand(String s){
 s.trim();if(s.equalsIgnoreCase("STOP")){autoStopCommand();return;}if(!stripChecksum(s))return;
 if(s.equalsIgnoreCase("STOP")){autoStopCommand();return;}if(!s.startsWith("DRV,"))return;
 int c1=s.indexOf(','),c2=s.indexOf(',',c1+1);if(c1<0||c2<0)return;int t,st;
 if(!parseIntStrict(s.substring(c1+1,c2),t)||!parseIntStrict(s.substring(c2+1),st))return;
 if(t < -1000 || t > 1000 || st < -1000 || st > 1000)return;
 autoThrottle=t/1000.0f;autoSteering=st/1000.0f;autoCommandValid=true;lastAutoCommandMs=millis();
}
void readAutoSerial(){
 while(Serial2.available()){char c=Serial2.read();if(c=='\n'){if(autoDiscard)autoDiscard=false;else processAutoCommand(autoBuffer);autoBuffer="";}
 else if(c!='\r'){if(autoDiscard){}else if(autoBuffer.length()<80)autoBuffer+=c;else{autoBuffer="";autoDiscard=true;}}}
}

bool readAutoSwitch(){
 bool raw=(digitalRead(PIN_AUTO_SWITCH)==LOW);uint32_t now=millis();
 if(raw!=switchCandidate){switchCandidate=raw;switchCandidateMs=now;}
 else if(raw!=switchStable&&now-switchCandidateMs>=SWITCH_DEBOUNCE_MS)switchStable=raw;return switchStable;
}
const char* sonarText(const SonarState& s){return s.fault?"FAULT":(s.blocked?"BLOCK":"clear");}
void printStatus(bool autoMode){
 uint32_t now=millis();if(now-lastStatusMs<STATUS_INTERVAL_MS)return;lastStatusMs=now;
 Serial.printf("mode=%s ramp=%+.2f/%+.2f rcArmed=%d front=%s(%.0fcm,%s) rear=%s(%.0fcm,%s)\n",
 autoMode?"AUTO":"RC",rampThrottle,rampSteering,rcArmed?1:0,
 sonarText(frontSonar),frontSonar.distanceCm,frontSonar.armed?"armed":"UNARMED",
 sonarText(rearSonar),rearSonar.distanceCm,rearSonar.armed?"armed":"UNARMED");
}

void setup(){
 Serial.begin(115200);Serial2.begin(115200,SERIAL_8N1,PIN_AUTO_RX,PIN_AUTO_TX);
 pinMode(PIN_MOTOR_ENABLE,OUTPUT);digitalWrite(PIN_MOTOR_ENABLE,LOW);
 pinMode(PIN_RC_STEERING,INPUT);pinMode(PIN_RC_THROTTLE,INPUT);pinMode(PIN_AUTO_SWITCH,INPUT_PULLUP);
 pinMode(PIN_FRONT_TRIG,OUTPUT);pinMode(PIN_FRONT_ECHO,INPUT);pinMode(PIN_REAR_TRIG,OUTPUT);pinMode(PIN_REAR_ECHO,INPUT);
 digitalWrite(PIN_FRONT_TRIG,LOW);digitalWrite(PIN_REAR_TRIG,LOW);
 const uint8_t pins[]={PIN_FL_RPWM,PIN_FL_LPWM,PIN_FR_RPWM,PIN_FR_LPWM,PIN_RL_RPWM,PIN_RL_LPWM,PIN_RR_RPWM,PIN_RR_LPWM};
 for(uint8_t p:pins)configurePwmPin(p);haltAll();
 attachInterrupt(digitalPinToInterrupt(PIN_RC_STEERING),steeringISR,CHANGE);
 attachInterrupt(digitalPinToInterrupt(PIN_RC_THROTTLE),throttleISR,CHANGE);
 for(uint8_t i=0;i<6;i++){updateSonars();delay(75);}
 switchStable=switchCandidate=(digitalRead(PIN_AUTO_SWITCH)==LOW);switchCandidateMs=millis();
 previousAutoMode=switchStable;modeChangeMs=millis();autoCommandValid=false;rcArmed=false;rcNeutralTiming=false;
 delay(500);digitalWrite(PIN_MOTOR_ENABLE,HIGH);
#if CORE_V3
 esp_task_wdt_config_t wdtCfg={};wdtCfg.timeout_ms=WDT_TIMEOUT_S*1000;wdtCfg.idle_core_mask=0;wdtCfg.trigger_panic=true;
 esp_task_wdt_reconfigure(&wdtCfg);
#else
 esp_task_wdt_init(WDT_TIMEOUT_S,true);
#endif
 esp_task_wdt_add(NULL);
}

void loop(){
 esp_task_wdt_reset();readAutoSerial();updateSonars();
 const bool autoMode=readAutoSwitch();
 if(autoMode!=previousAutoMode){
  haltAll();modeChangeMs=millis();previousAutoMode=autoMode;autoThrottle=autoSteering=0.0f;autoCommandValid=false;
  rcArmed=false;rcNeutralTiming=false;
 }
 printStatus(autoMode);if(millis()-modeChangeMs<MODE_CHANGE_STOP_MS){haltAll();delay(5);return;}
 if(autoMode){
  if(!autoCommandValid||millis()-lastAutoCommandMs>AUTO_TIMEOUT_MS){haltAll();delay(5);return;}
  driveWithRamp(autoThrottle,autoSteering);
 }else{
  uint16_t sp,tp;uint32_t sLast,tLast;noInterrupts();
  sp=steeringPulseUs;tp=throttlePulseUs;sLast=steeringLastPulseMs;tLast=throttleLastPulseMs;interrupts();
  uint32_t now=millis();
  if(now-sLast>=RC_TIMEOUT_MS||now-tLast>=RC_TIMEOUT_MS){haltAll();rcArmed=false;rcNeutralTiming=false;delay(5);return;}
  float s=convertCenteredRc(sp,RC_STEER_LEFT_US,RC_STEER_CENTER_US,RC_STEER_RIGHT_US,RC_STEER_DEADBAND_US);
  float t=convertCenteredRc(tp,RC_THROTTLE_REV_US,RC_THROTTLE_NEUTRAL_US,RC_THROTTLE_FWD_US,RC_THROTTLE_DEADBAND_US);
  if(!rcArmed){
   if(t==0.0f&&s==0.0f){if(!rcNeutralTiming){rcNeutralTiming=true;rcNeutralStartMs=now;}else if(now-rcNeutralStartMs>=RC_NEUTRAL_HOLD_MS)rcArmed=true;}
   else rcNeutralTiming=false;if(!rcArmed){haltAll();delay(5);return;}
  }
  driveWithRamp(t,s);
 }
 delay(5);
}
