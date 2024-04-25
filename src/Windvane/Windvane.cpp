#include "Windvane.h"


// Retrieving wind direction from rotary encoder "tick" position
int getWindDir(RotaryEncoder *encoder){
  int pos = abs((int)encoder->getPosition()) % 1190; //1190 full rotation
  int angle;
  if ((int)encoder->getDirection() <0) {
    angle = 360 - map(pos,0,1190,0,360);
    }
  else {angle = map(pos,0,1190,0,360);}
  return angle;
}

float getWinchAngle(float windAngle){
    if     (windAngle >=0 && windAngle <= 20  ) { return 72; }
    else if(windAngle >20 && windAngle <= 60  ) { return 72; }
    else if(windAngle >60 && windAngle <= 70  ) { return 72; }
    else if(windAngle >70 && windAngle <= 110 ) { return 75; }
    else if(windAngle >110 && windAngle <= 140) { return 80; }
    else if(windAngle >140 && windAngle <= 220) { return 110;}
    else if(windAngle >220 && windAngle <= 250) { return 80; }
    else if(windAngle >250 && windAngle <= 290) { return 75; }
    else if(windAngle >290 && windAngle <= 300) { return 72; }
    else if(windAngle >300 && windAngle <= 340) { return 72; }
    else                                        { return 72; }
}
 