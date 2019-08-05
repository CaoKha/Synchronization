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

/* ParamForSyncReadInst_t
  A structure that contains the information needed for the parameters of the 'syncRead packet'.

  typedef struct ParamForSyncReadInst{
    uint16_t addr;
    uint16_t length;
    uint8_t id_count;
    InfoForSyncReadParam_t xel[DXL_MAX_NODE]; //refer to below.
  } ParamForSyncReadInst_t;

  typedef struct InfoForSyncReadParam{
    uint8_t id;
  } InfoForSyncReadParam_t;
*/

/* ParamForSyncWriteInst_t
  A structure that contains the information needed for the parameters of the 'syncWrite packet'.

  typedef struct ParamForSyncWriteInst{
    uint16_t addr;
    uint16_t length;
    uint8_t id_count;
    XelInfoForSyncWriteParam_t xel[DXL_MAX_NODE]; //refer to below.
  } ParamForSyncWriteInst_t;

  typedef struct XelInfoForSyncWriteParam{
    uint8_t id;
    uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
  } XelInfoForSyncWriteParam_t;
*/

/* RecvInfoFromStatusInst_t
  A structure used to receive data from multiple XELs.

  typedef struct RecvInfoFromStatusInst{
    uint8_t id_count;
    XelInfoForStatusInst_t xel[DXL_MAX_NODE]; //refer to below.
  } RecvInfoFromStatusInst_t;

  typedef struct XelInfoForStatusInst{
    uint8_t id;
    uint16_t length;
    uint8_t error;
    uint8_t data[DXL_MAX_NODE_BUFFER_SIZE];
  } XelInfoForStatusInst_t;
*/

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  
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

void loop() {
  // put your main code here, to run repeatedly:
  static int32_t velocity = 0;
  int32_t recv_velocity[2];

  // set value to data buffer for syncWrite
  velocity = velocity >= 200 ? -200 : velocity+10;
  memcpy(sync_write_param.xel[0].data, &velocity, sizeof(velocity));
  memcpy(sync_write_param.xel[1].data, &velocity, sizeof(velocity));

  // send command using syncWrite
  dxl.syncWrite(sync_write_param);
  delay(100);

  // Print the read data using SyncRead
  dxl.syncRead(sync_read_param, read_result);
  DEBUG_SERIAL.println("======= Sync Read =======");
  memcpy(&recv_velocity[0], read_result.xel[0].data, read_result.xel[0].length);
  memcpy(&recv_velocity[1], read_result.xel[1].data, read_result.xel[1].length);
  DEBUG_SERIAL.print("ID: ");DEBUG_SERIAL.print(read_result.xel[0].id);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Present Velocity: ");DEBUG_SERIAL.print(recv_velocity[0]);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Packet Error: ");DEBUG_SERIAL.print(read_result.xel[0].error);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Param Length: ");DEBUG_SERIAL.print(read_result.xel[0].length);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("ID: ");DEBUG_SERIAL.print(read_result.xel[1].id);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Present Velocity: ");DEBUG_SERIAL.print(recv_velocity[1]);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Packet Error: ");DEBUG_SERIAL.print(read_result.xel[1].error);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.print(", Param Length: ");DEBUG_SERIAL.print(read_result.xel[1].length);DEBUG_SERIAL.print(" ");
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();  
  delay(100);
}


