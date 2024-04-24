#include <Arduino.h>
#include "Windvane.h"
#include "Telemetry/Telemetry.h"

float winchAngle;
void setup() {
  teleSerial.begin(57600);
  encoder = new RotaryEncoder(WINDVANE_W, WINDVANE_G, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(WINDVANE_W), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WINDVANE_G), checkPosition, CHANGE);
}

void loop() {
   windAngle = getWindDir();
   winchAngle = getWinchAngle(windAngle);
   teleSerial.printf("Wind Angle: %.2f \t Winch Angle: %.2f\n", windAngle, winchAngle);
   delay(1000);
}