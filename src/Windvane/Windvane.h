#ifndef WINDVANE_H
#define WINDVANE_H

#include<RotaryEncoder.h>

#define WINDVANE_W PB13 
#define WINDVANE_G PA6

/* WIND VANE */
static float windAngle;

float getWinchAngle(float windAngle);
int getWindDir(RotaryEncoder *encoder);
void checkPosition();

#endif //WINDVANE_H