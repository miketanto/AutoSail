#include <Arduino.h>
#include <HardwareSerial.h>
#include "Telemetry/Telemetry.h"
#include "Windvane/Windvane.h"
#include <Servo.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <vector.h>

/*TELEMETRY*/
unsigned long prevTeleTime = 0;
unsigned long teleTime;
HardwareSerial teleSerial(PB10);

/*WINDVANE*/
RotaryEncoder *encoder = nullptr;

// Rotary Encoder ISR
void checkPosition()
{
  encoder->tick();
}

/** SERVO **/
#define WINCH_SERVO PB14
#define RUDDER_SERVO PB15
Servo winchServo, rudderServo; 
volatile int val[4]; volatile long start[4]; volatile long pulses[4]; volatile long pulseWidth[4];

/**IMU **/
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire);
static float roll, _heading;
std::vector<float> getHeadingRoll(Adafruit_BNO055 &bno);
std::vector<float> imuData;

/**Remote Definitions*/
#define CH1 PA0
#define CH3 PA1
#define CH5 PA2
#define CH6 PA3
/**Remote Functions*/
void setRudder();
void setWinch();
void setModeISR();
void setBase();
void (*setVal[4])() = { setWinch, setRudder, setModeISR,setBase};
void PulseTimer(int ISN);
/**Remote Interrupts*/
void ISR_WINCH()  { PulseTimer(0);}
void ISR_RUDDER() { PulseTimer(1);}
void ISR_MODE()   { PulseTimer(2);}
void ISR_BASE()   { PulseTimer(3);}
/* CALIBRATED RANGE OF PWM READS FOR EACH CHANNEL  */
// CH1,CH3,CH5,CH6
int range[4][2] = {{950,2050},{1000,2000},{950,2010},{900,2100}};
int servoRange[2][2] = {{37,165},{72,110}};
static double _lat,_lng, base_lat, base_lng;
/* TEMP VALS, CONTROL LIB TO BE ADDED */
static float _winchAngle, _rudderAngle, _mode, _base;
static int phi;
volatile float newPhi;
volatile float _dev = 85;


/**PID Control Variables*/
double getRudderAngle(float deviation, float period);
static float distanceTo, period,prevTime;
static float desiredHeading = 0; 
const float PID_period = 100; //in ms
float PID_prevIntegral;
const long Kp = 3.1;
const float Td = 0.1;
const float Ti = 3;
const float Ki = Kp / Ti;
const float Kd = Kp * Td;
float heelingPhi;
int autoLoop;


/** MAIN PROGRAM **/
void setup() {
   /* TELEMETRY SETUP */
    teleSerial.begin(57600);

    /*WINDVANE SETUP*/

    encoder = new RotaryEncoder(WINDVANE_W, WINDVANE_G, RotaryEncoder::LatchMode::TWO03);
    attachInterrupt(digitalPinToInterrupt(WINDVANE_W), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(WINDVANE_G), checkPosition, CHANGE);

    /*NEW IMU SETUP*/
   bno.begin();

   /**Servo Setup*/
   rudderServo.attach(RUDDER_SERVO);
   winchServo.attach(WINCH_SERVO);
   _winchAngle = 90; _rudderAngle = 45;
    winchServo.write(_winchAngle);
    rudderServo.write(_rudderAngle);

   _mode = 0;
   autoLoop = 0;
    pinMode(CH1, INPUT_PULLUP);
    pinMode(CH3, INPUT_PULLUP);
    pinMode(CH5, INPUT_PULLUP);
    pinMode(CH6, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CH1),ISR_WINCH,CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH3),ISR_RUDDER,CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH5),ISR_MODE,CHANGE);
    attachInterrupt(digitalPinToInterrupt(CH6),ISR_BASE,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
    imuData = getHeadingRoll(bno);
    _heading = imuData[0];
    roll = imuData[1];
    windAngle = getWindDir(encoder);

    if(_mode == 0){
        autoLoop = 0;
        rudderServo.write(_rudderAngle);
        winchServo.write(_winchAngle);
    }else if(_mode == 1){
        if(autoLoop == 0){
            desiredHeading = _heading; 
            PID_prevIntegral = 0;
            prevTime = 0;
            heelingPhi = 0;
            autoLoop++;
        }else{
            float deviation = desiredHeading - _heading;
            if (deviation <= -180) {deviation = 360 + deviation;}
            else if (deviation >= 180) {deviation = -(360 - deviation);}
            
            unsigned long currentTime = millis();
            double newRudderAngle;
            // Output from PID is RELATIVE rudder angle between -45, 45
            if (currentTime - prevTime >= PID_period) {
                newRudderAngle = constrain (getRudderAngle(deviation, PID_period / 1000), -45,45);
                prevTime = currentTime;
            }
            _rudderAngle = map((int)newRudderAngle, -45, 45, 40,165);
            _winchAngle = getWinchAngle(windAngle);
            
            //Heeling Angle Adjustment
            if ((abs(roll) < 160) && (abs(roll) > 115)){ 
            if (heelingPhi >= (servoRange[1][1] - servoRange[1][0])){ heelingPhi = (servoRange[1][1] - servoRange[1][0]);}
            else {heelingPhi = heelingPhi + 0.02;}}
            else { 
                if (heelingPhi <= 0.08) { heelingPhi = 0; }
                else { heelingPhi = heelingPhi - 0.1;}
            }

            int tempWinch = _winchAngle + heelingPhi;
            _winchAngle = min(servoRange[1][1],tempWinch);
            
            rudderServo.write(_rudderAngle);
            winchServo.write(_winchAngle);
        }
    }
    teleTime = millis();
    printTelemetry(
        teleSerial,
        _mode, teleTime, prevTeleTime,
        0, 0, windAngle, 
        _winchAngle, _rudderAngle, 1, 
        _heading, 1, roll, 0, 
        0, base_lat, base_lng
    );
}
/** MAIN PROGRAM*/















double getRudderAngle(float deviation, float period){
    // PI CONTROLLER
    float integral  = PID_prevIntegral + (deviation * period);
    
    double newAngle = Kp*deviation + (Ki * integral);

    if (newAngle >= 45 || newAngle <= - 45 ){ integral  = PID_prevIntegral; }

    PID_prevIntegral = integral;

    return newAngle;
    
}
/* RUDDER IS CALIBRATED TO EVEN RANGE W RSPT TO BOAT (36,165) */
void setRudder(){
  if (_mode == 0){
    float newAngle = map(val[1],range[1][0],range[1][1],servoRange[0][0],servoRange[0][1]);
    if (newAngle < (_rudderAngle - 7.7) || newAngle > (_rudderAngle + 7.7)  ){
      _rudderAngle = newAngle;
     }
  }
}

/* VALUE GOES TO MAX WHEN REMOTE TURNS OFF -> FIX */
void setWinch(){
  /* Map PWM into range: -3,-2,-1,0,1,2,3 (multiplier for adjusting tightness) */
  newPhi = map(val[0],range[0][0],range[0][1],-3,4);
  float newAngle;
  if (newPhi > 0){newAngle = constrain(_winchAngle + ( (newPhi*newPhi) / 100),servoRange[1][0],servoRange[1][1]);}
  else {newAngle = constrain(_winchAngle -( (newPhi*newPhi) / 100),servoRange[1][0],servoRange[1][1]);}
  _winchAngle = newAngle;
}

void setModeISR(){
  /* MANUAL */
  if(val[2] <1400){
    _mode = 0;
  }
  /* RETURN TO BASE */
  else if (val[2] > 1600){
    _mode = 2;
  }
  /* AUTONOMOUS */
  else{
    _mode = 1;
  }
}

void setBase(){
  /* USER CAN ONLY SET BASE IN MANUAL OR AUTONOMOUS MODE */
  if (_mode != 2){
    if(val[3] < 1500){ _base = 0; }
    else { if(_base ==0) { base_lat = _lat; base_lng = _lng; }
       _base = 1;
       }  
    }
}

void PulseTimer(int ISN){
  pulses[ISN] = micros();
  if (pulses[ISN] > start[ISN]){
    pulseWidth[ISN] = pulses[ISN] - start[ISN];
    start[ISN] = pulses[ISN];
  }
  
  if (range[ISN][0] < pulseWidth[ISN] && pulseWidth[ISN] < range[ISN][1]){
    val[ISN] = pulseWidth[ISN];
    setVal[ISN]();
  }
}

std::vector<float> getHeadingRoll(Adafruit_BNO055 &bno){
    imu::Vector<3> from(1,0,0);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    imu::Vector<3> E = mag.cross(acc);
    E.normalize();

    imu::Vector<3> N = acc.cross(E);
    N.normalize();

    float heading = atan2(E.dot(from), N.dot(from)) * 180 / PI;  
    if (heading < 0) heading += 360;
    float roll  = atan2(acc[1],acc[2]) * 57.2957;
    std::vector<float> ret(heading, roll);
    return ret;
}
