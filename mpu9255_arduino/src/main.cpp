#include "MPU9250.h" // include MPU9255 library
#define Serial SerialUSB

MPU9250_<TwoWire, AFS::A2G, GFS::G250DPS, MFS::M14BITS> mpu;
unsigned long microsPerReading, microsPrevious;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  mpu.setup();
  delay(5000);
  mpu.setMagneticDeclination(-0.17); //Carquefou
  // calibrate anytime you want to
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();
  mpu.printCalibration();
  microsPerReading = 1000000 / 200;
  microsPrevious = micros();
}

void loop()
{
  unsigned long microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading)
  {
    mpu.update();
    mpu.printRawData();
    // unsigned long elapsed = micros() - microsNow;
    // Serial.println(elapsed);
    microsPrevious = microsPrevious + microsPerReading;
  }
}
