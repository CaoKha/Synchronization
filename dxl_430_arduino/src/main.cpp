/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>

#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); //RX,TX
#define DEBUG_SERIAL soft_serial
#elif ARDUINO_AVR_MEGA2560
#define DEBUG_SERIAL Serial1
#else
#define DEBUG_SERIAL Serial
#endif

#define WHEEL_RADIUS 0.033    // meter
#define WHEEL_SEPARATION 0.16 // meter (BURGER => 0.16, WAFFLE => 0.287)
// #define ROBOT_LENGTH                    0.165     // meter

#define WHEEL_POS_FROM_CENTER_X_1 -0.100 // meter
#define WHEEL_POS_FROM_CENTER_Y_1 -0.128 // meter
#define WHEEL_POS_FROM_CENTER_X_2 0.100  // meter
#define WHEEL_POS_FROM_CENTER_Y_2 -0.128 // meter
#define WHEEL_POS_FROM_CENTER_X_3 -0.100 // meter
#define WHEEL_POS_FROM_CENTER_Y_3 0.128  // meter
#define WHEEL_POS_FROM_CENTER_X_4 0.100  // meter
#define WHEEL_POS_FROM_CENTER_Y_4 0.128  // meter

#define ENCODER_MIN -2147483648 // raw
#define ENCODER_MAX 2147483648  // raw

#define VELOCITY_CONSTANT_VAULE 1263.632956882 // V = r * w = r * RPM * 0.10472          \
                                               //   = 0.033 * 0.229 * Goal RPM * 0.10472 \
                                               // Goal RPM = V * 1263.632956882

#define CONTROL_PERIOD 8000

#define MAX_LINEAR_VELOCITY 0.22  // m/s
#define MAX_ANGULAR_VELOCITY 2.84 // rad/s
#define VELOCITY_LINEAR_X 0.01    // m/s
#define VELOCITY_ANGULAR_Z 0.1    // rad/s
#define SCALE_VELOCITY_LINEAR_X 1
#define SCALE_VELOCITY_ANGULAR_Z 1

#define DEG2RAD(x) (x * 0.01745329252) // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI
#define LIMIT_X_MAX_VELOCITY 240

double linear_x = 0.0;
double angular_z = 0.0;
double goal_linear_velocity = 0.0;
double goal_angular_velocity = 0.0;
double const_cmd_vel = 0.2;
const float DXL_PROTOCOL_VERSION = 2.0;
ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;

DynamixelShield dxl;


void setup()
{
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  dxl.scan();

  // fill the members of structure for syncWrite
  sync_write_param.addr = 104; //Goal Velocity on X serise
  sync_write_param.length = 4;
  sync_write_param.xel[0].id = 1;
  sync_write_param.xel[1].id = 3;
  sync_write_param.id_count = 2;

  // fill the members of structure for syncRead
  sync_read_param.addr = 128; //Present Velocity on X serise
  sync_read_param.length = 4;
  sync_read_param.xel[0].id = 1;
  sync_read_param.xel[1].id = 3;
  sync_read_param.id_count = 2;

  dxl.torqueOff(1);
  dxl.setOperatingMode(1, OP_VELOCITY);
  dxl.torqueOn(1);

  dxl.torqueOff(3);
  dxl.setOperatingMode(3, OP_VELOCITY);
  dxl.torqueOn(3);
}

void loop()
{
  // put your main code here, to run repeatedly:
  int32_t recv_velocity[2];
  // set value to data buffer for syncWrite
  // velocity = velocity >= 200 ? -200 : velocity+10;
  uint8_t received_data;
  double wheel1_spd_cmd, wheel2_spd_cmd;
  double lin_vel1, lin_vel2;
  while (DEBUG_SERIAL.available() == 0)
    ;
  received_data = DEBUG_SERIAL.read();

  switch (received_data)
  {
  case '8':
    linear_x += VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    break;
  case '2':
    linear_x -= VELOCITY_LINEAR_X * SCALE_VELOCITY_LINEAR_X;
    break;
  case '4':
    angular_z += VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    break;
  case '6':
    angular_z -= VELOCITY_ANGULAR_Z * SCALE_VELOCITY_ANGULAR_Z;
    break;
  case '7':
    linear_x = const_cmd_vel;
    angular_z = 0.0;
    break;
  case '9':
    linear_x = 0.0;
    angular_z = 0.0;
    break;
  }

  if (linear_x > MAX_LINEAR_VELOCITY)
    linear_x = MAX_LINEAR_VELOCITY;

  if (angular_z > MAX_ANGULAR_VELOCITY)
    angular_z = MAX_ANGULAR_VELOCITY;

  goal_linear_velocity = linear_x;
  goal_angular_velocity = angular_z;

  wheel1_spd_cmd = goal_linear_velocity - (sqrt(WHEEL_POS_FROM_CENTER_X_1 * WHEEL_POS_FROM_CENTER_X_1 + WHEEL_POS_FROM_CENTER_Y_1 * WHEEL_POS_FROM_CENTER_Y_1) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_1 / WHEEL_POS_FROM_CENTER_X_1));
  wheel2_spd_cmd = goal_linear_velocity + (sqrt(WHEEL_POS_FROM_CENTER_X_2 * WHEEL_POS_FROM_CENTER_X_2 + WHEEL_POS_FROM_CENTER_Y_2 * WHEEL_POS_FROM_CENTER_Y_2) * goal_angular_velocity) * cos(atan(WHEEL_POS_FROM_CENTER_Y_2 / WHEEL_POS_FROM_CENTER_X_2));

  lin_vel1 = wheel1_spd_cmd * VELOCITY_CONSTANT_VAULE;

  if (lin_vel1 > LIMIT_X_MAX_VELOCITY)
    lin_vel1 = LIMIT_X_MAX_VELOCITY;
  else if (lin_vel1 < -LIMIT_X_MAX_VELOCITY)
    lin_vel1 = -LIMIT_X_MAX_VELOCITY;

  lin_vel2 = -1 * wheel2_spd_cmd * VELOCITY_CONSTANT_VAULE;
  if (lin_vel2 > LIMIT_X_MAX_VELOCITY)
    lin_vel2 = LIMIT_X_MAX_VELOCITY;
  else if (lin_vel2 < -LIMIT_X_MAX_VELOCITY)
    lin_vel2 = -LIMIT_X_MAX_VELOCITY;

  int8_t velocity1 = lin_vel1;
  int8_t velocity2 = lin_vel2;

  memcpy(sync_write_param.xel[0].data, &velocity1, sizeof(velocity1));
  memcpy(sync_write_param.xel[1].data, &velocity2, sizeof(velocity2));

  // send command using syncWrite
  dxl.syncWrite(sync_write_param);
  delay(100);

  // Print the read data using SyncRead
  dxl.syncRead(sync_read_param, read_result);
  DEBUG_SERIAL.println("======= Sync Read =======");
  memcpy(&recv_velocity[0], read_result.xel[0].data, read_result.xel[0].length);
  memcpy(&recv_velocity[1], read_result.xel[1].data, read_result.xel[1].length);
  DEBUG_SERIAL.print("ID: ");
  DEBUG_SERIAL.print(read_result.xel[0].id);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Present Velocity: ");
  DEBUG_SERIAL.print(recv_velocity[0]);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Packet Error: ");
  DEBUG_SERIAL.print(read_result.xel[0].error);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Param Length: ");
  DEBUG_SERIAL.print(read_result.xel[0].length);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("ID: ");
  DEBUG_SERIAL.print(read_result.xel[1].id);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Present Velocity: ");
  DEBUG_SERIAL.print(recv_velocity[1]);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Packet Error: ");
  DEBUG_SERIAL.print(read_result.xel[1].error);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Param Length: ");
  DEBUG_SERIAL.print(read_result.xel[1].length);
  DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(lin_vel1);
  DEBUG_SERIAL.println(lin_vel2);
  DEBUG_SERIAL.println(velocity1);
  DEBUG_SERIAL.println(velocity2);
  DEBUG_SERIAL.println();
  delay(100);
}
