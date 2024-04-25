#ifndef TELEMETRY_H
#define TELEMETRY_H

#define TELE_PERIOD 1000
#include <HardwareSerial.h>

void printTelemetry(
    HardwareSerial& teleSerial,
    float _mode, unsigned long teleTime, unsigned long &prevTeleTime,
    double _lat, double _lng, float windAngle, 
    float _winchAngle, float _rudderAngle, float desiredHeading, 
    float _heading, float _course, float roll, float _knots, 
    float distanceTo, double base_lat, double base_lng
);

#endif //TELEMETRY_H