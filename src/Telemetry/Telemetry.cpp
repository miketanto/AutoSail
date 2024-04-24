#include <Arduino.h>
#include <math.h>
#include "Telemetry.h"

/* TELEMETRY INFO */
void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    teleSerial.print (int(val));  //prints the int part
    teleSerial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val)- val ) * precision;
    teleSerial.print(frac,DEC) ;
} 

void printDoublenewL( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

    teleSerial.print (int(val));  //prints the int part
    teleSerial.print("."); // print the decimal point
    unsigned int frac;
    if(val >= 0)
        frac = (val - int(val)) * precision;
    else
        frac = (int(val)- val ) * precision;
    teleSerial.println(frac,DEC) ;
} 

void printTelemetry(
    float _mode, unsigned long teleTime, unsigned long &prevTeleTime,
    double _lat, double _lng, float windAngle, 
    float _winchAngle, float _rudderAngle, float desiredHeading, 
    float _heading, float _course, float roll, float _knots, 
    float distanceTo, double base_lat, double base_lng
){
    if (teleTime >= prevTeleTime + telePeriod) {

        if(_mode == 0)teleSerial.print("MANUAL ");
        else if(_mode == 1)teleSerial.print("AUTO ");
        else teleSerial.print("RETURN TO BASE");

        teleSerial.print(" lati ");
        printDouble(_lat,1000000);
        teleSerial.print(" lngi ");
        //teleSerial.print(_lng);
        printDouble(_lng,1000000);
        teleSerial.print(" windAngle ");
        teleSerial.print(windAngle);
        teleSerial.print(" winchAngle ");
        teleSerial.print(_winchAngle);
        teleSerial.print(" rudderAngle ");
        teleSerial.print(_rudderAngle);
        teleSerial.print(" desiredHeading ");
        teleSerial.print(desiredHeading);
        teleSerial.print(" heading ");
        teleSerial.print(_heading);
        teleSerial.print(" course ");
        teleSerial.print(_course);
        teleSerial.print(" roll ");
        teleSerial.print(roll);
        teleSerial.print(" knots ");
        teleSerial.print(_knots);
        teleSerial.print(" distanceTobase ");
        teleSerial.print(distanceTo);
        teleSerial.print(" baselat ");
        printDouble(base_lat,1000000);
        teleSerial.print(" baselng ");
        printDoublenewL(base_lng,1000000);
        prevTeleTime = teleTime;
    }
}
