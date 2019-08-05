#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define Serial SerialUSB
Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long microsPerReading, microsPrevious;

void setup(void)
{
  Serial.begin(460800);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  microsPerReading = 1000000 / 100;
  microsPrevious = micros();
}

void loop(void)
{
  unsigned long microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading)
  {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Quaternion quat = bno.getQuat();
    Serial.print(accel.x());
    Serial.print(";");
    Serial.print(accel.y());
    Serial.print(";");
    Serial.print(accel.z());
    Serial.print(";");
    Serial.print(gyro.x());
    Serial.print(";");
    Serial.print(gyro.y());
    Serial.print(";");
    Serial.print(gyro.z());
    Serial.print(";");
    Serial.print(mag.x());
    Serial.print(";");
    Serial.print(mag.y());
    Serial.print(";");
    Serial.print(mag.z());
    Serial.print(";");
    Serial.print(quat.x(),4);
    Serial.print(";");
    Serial.print(quat.y(),4);
    Serial.print(";");
    Serial.print(quat.z(),4);
    Serial.print(";");
    Serial.println(quat.w(),4);
    // unsigned long elapsed = micros() - microsNow;
    // Serial.println(elapsed);
    microsPrevious = microsPrevious + microsPerReading;
  }
}
