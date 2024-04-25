
// /* CALIBRATION OF ECOMPASS */
// unsigned long calibrationTime = 15000; //in ms

// /* Setup pointers to eCompass module PB6 (SCL), PB7 (SDA) -> DONE IN WIRE.BEGIN() */
// LSM303 compass;
// LSM303::vector<int16_t> running_min = {32767,32767,32767}, running_max = {-32768,-32768,-32768};

//  Wire.begin();
//   compass.init();
//   compass.enableDefault();
//   compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
//   compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
   
   
//    compass.read();
//         _heading = compass.heading();
//         roll = atan2(compass.a.y,compass.a.z) * 57.295