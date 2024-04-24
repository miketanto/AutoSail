#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <HardwareSerial.h>
HardwareSerial teleSerial(PB10);

unsigned long prevTeleTime = 0;
const int telePeriod = 1000;
unsigned long teleTime;

void printTelemetry(
    float _mode, unsigned long teleTime, unsigned long &prevTeleTime,
    double _lat, double _lng, float windAngle, 
    float _winchAngle, float _rudderAngle, float desiredHeading, 
    float _heading, float _course, float roll, float _knots, 
    float distanceTo, double base_lat, double base_lng
);

#endif TELEMETRY_H