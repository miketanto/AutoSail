// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <imumaths.h>
// #include "mbed.h"
// #include <Wire.h>

// #define BNO055_SAMPLERATE_DELAY_MS (100)

// Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A, &Wire);

// bno.begin();

// float getHeading(Adafruit_BNO055 &bno){
//     imu::Vector<3> from(1,0,0);
//     imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//     imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

//     imu::Vector<3> E = mag.cross(acc);
//     E.normalize();

//     imu::Vector<3> N = acc.cross(E);
//     N.normalize();

//     float heading = atan2(E.dot(from), N.dot(from)) * 180 / PI;  
//     if (heading < 0) heading += 360;
//     return heading;
// }
