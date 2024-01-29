// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Based on
//   https://github.com/YDLIDAR/lidarCar/

#include "LDS_YDLIDAR_X3_PRO.h"

void LDS_YDLIDAR_X3_PRO::init() {
  ring_start_ms[0] = ring_start_ms[1] = 0;
  enableMotor(false);
}

LDS::result_t LDS_YDLIDAR_X3_PRO::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, "YDLIDAR X3 PRO");
  postInfo(INFO_SAMPLING_RATE, String(getSamplingRateHz()));
  postInfo(INFO_DEFAULT_TARGET_SCAN_FREQ_HZ, String(getTargetScanFreqHz()));
  return LDS::RESULT_OK;
}

uint32_t LDS_YDLIDAR_X3_PRO::getSerialBaudRate() {
  return 115200;
}

float LDS_YDLIDAR_X3_PRO::getTargetScanFreqHz() {
  return 7;
}

int LDS_YDLIDAR_X3_PRO::getSamplingRateHz() {
  return 4000;
}

void LDS_YDLIDAR_X3_PRO::stop() {
  enableMotor(false);
}

void LDS_YDLIDAR_X3_PRO::enableMotor(bool enable) {
  motor_enabled = enable;

  setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);

  // Entire LDS on/off
  setMotorPin(enable ? VALUE_HIGH : VALUE_LOW, LDS_MOTOR_PWM_PIN);
}

LDS::result_t LDS_YDLIDAR_X3_PRO::waitScanDot() {
  static int recvPos = 0;
  static uint8_t package_Sample_Num = 0;
  static int package_recvPos = 0;
  static int package_sample_sum = 0;
  static int currentByte = 0;

  static node_package package;
  static uint8_t *packageBuffer = (uint8_t*)&package.package_Head;

  static uint16_t package_Sample_Index = 0;
  static float IntervalSampleAngle = 0;
  static float IntervalSampleAngle_LastPackage = 0;
  static uint16_t FirstSampleAngle = 0;
  static uint16_t LastSampleAnglePrev = 0;
  static uint16_t LastSampleAngle = 0;
  static uint16_t CheckSum = 0;

  static uint16_t CheckSumCal = 0;
  static uint16_t SampleNumlAndCTCal = 0;
  static uint16_t LastSampleAngleCal = 0;
  static bool CheckSumResult = true;
  static uint16_t Valu8Tou16 = 0;

  static uint8_t state = 0;
  static bool scan_completed = false;

  switch(state) {
    case 1:
      goto state1;
    case 2:
      goto state2;
  }

  if (package_Sample_Index == 0) {
    
    package_Sample_Num = 0;
    package_recvPos = 0;
    recvPos = 0; // Packet start

    // Read in 10-byte packet header
    while (true) {
state1:
      currentByte = readSerial();
      if (currentByte<0) {
        state = 1;
        return ERROR_NOT_READY;
      }

      switch (recvPos) {
        case 0: // 0xAA 1st byte of package header
          if (currentByte != (PH&0xFF))
            continue;
          break;
        case 1: // 0x55 2nd byte of package header
          CheckSumCal = PH;
          if (currentByte != (PH>>8)) {
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          SampleNumlAndCTCal = currentByte;
          if ((currentByte != CT_NORMAL) && (currentByte != CT_RING_START)) { 
            recvPos = 0;
            continue;
          }
          break;
        case 3:
          SampleNumlAndCTCal += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          package_Sample_Num = currentByte;
          break;
        case 4:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            FirstSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 5:
          FirstSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle>>1;
          break;
        case 6:
          if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
            LastSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 7:
          LastSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle>>1;
          if (package_Sample_Num == 1) {
            IntervalSampleAngle = 0;
          } else {
            if (LastSampleAngle < FirstSampleAngle) {
              if ((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)) {
                IntervalSampleAngle = ((float)(23040 + LastSampleAngle
                  - FirstSampleAngle))/(package_Sample_Num-1);
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              } else {
                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }
            } else {
              IntervalSampleAngle = ((float)(LastSampleAngle -FirstSampleAngle))/(package_Sample_Num-1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }
          break;
        case 8:
          CheckSum = currentByte; 
          break;
        case 9:
          CheckSum += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          break;
      }     
      packageBuffer[recvPos++] = currentByte;

      if (recvPos == PACKAGE_PAID_BYTES ) {
        package_recvPos = recvPos;
        break;
      }
    }

    // Check buffer overflow
    if (package_Sample_Num > PACKAGE_SAMPLE_MAX_LENGTH)
      return ERROR_INVALID_PACKET;

    if (PACKAGE_PAID_BYTES == recvPos) {
      // Packet header looks valid size-wise
      recvPos = 0;
      package_sample_sum = package_Sample_Num<<1;

      // Read in samples, 2 bytes each
      while (true) {
state2:
        currentByte = readSerial();
        if (currentByte < 0){
          state = 2;
          return ERROR_NOT_READY;
        }

        if ((recvPos & 1) == 1) {
          Valu8Tou16 += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= Valu8Tou16;
        } else {
          Valu8Tou16 = currentByte; 
        }
                    
        packageBuffer[package_recvPos+recvPos] = currentByte;          
        recvPos++;
        if (package_sample_sum == recvPos) {
          package_recvPos += recvPos;
          break;
        }
      }

      if (package_sample_sum != recvPos) {
        state = 0;
        return ERROR_INVALID_PACKET;
      }
    } else {
      // Packet length is off
      state = 0;
      return ERROR_INVALID_PACKET;
    }
    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    if (CheckSumCal != CheckSum){  
      CheckSumResult = false;
    } else {
      CheckSumResult = true;
    }
  }

  if (CheckSumResult) {
    scan_completed = FirstSampleAngle <= LastSampleAnglePrev;
    LastSampleAnglePrev = LastSampleAngle;
    if (scan_completed)
      markScanTime();
    postPacket(packageBuffer, PACKAGE_PAID_BYTES+package_sample_sum, scan_completed);
  }

  // Process the buffered packet
  while(true) {
    uint8_t package_CT;
    node_info node;
  
    package_CT = package.package_CT;    
    if (package_CT == CT_NORMAL) {
      node.sync_quality = NODE_DEFAULT_QUALITY + NODE_NOT_SYNC;
    } else {
      node.sync_quality = NODE_DEFAULT_QUALITY + NODE_SYNC;
    }
  
    if (CheckSumResult == true) {
      int32_t AngleCorrectForDistance;
      node.distance_q2 = package.packageSampleDistance[package_Sample_Index];
            
      if (node.distance_q2/4 != 0) {
        AngleCorrectForDistance = (int32_t)((atan(((21.8f*(155.3f
          - (node.distance_q2*0.25f)) )/155.3f)/(node.distance_q2*0.25f)))*3666.93f);
      } else {
        AngleCorrectForDistance = 0;    
      }

      float sampleAngle = IntervalSampleAngle*package_Sample_Index;

      if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0) {
        node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle
          + AngleCorrectForDistance + 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)
          + LIDAR_RESP_MEASUREMENT_CHECKBIT;
      } else {
        if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
          node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle
            + AngleCorrectForDistance - 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)
            + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } else {
          node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle
            + AngleCorrectForDistance))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)
            + LIDAR_RESP_MEASUREMENT_CHECKBIT;
        } 
      }
    } else {
      // Invalid checksum
      node.sync_quality = NODE_DEFAULT_QUALITY + NODE_NOT_SYNC;
      node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
      node.distance_q2 = 0;
      package_Sample_Index = 0;
      state = 0;
      return ERROR_CRC;
    }
  
    // Dump out processed data
    float point_distance_mm = node.distance_q2*0.25f;
    float point_angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    uint8_t point_quality = (node.sync_quality>>LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
    //bool point_startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);

    postScanPoint(point_angle, point_distance_mm, point_quality, scan_completed);
    scan_completed = false;

    // Dump finished?
    package_Sample_Index++;
    uint8_t nowPackageNum = package.nowPackageNum;  
    if (package_Sample_Index >= nowPackageNum) {
      package_Sample_Index = 0;
      break;
    }
  }
  state = 0;
  return LDS::RESULT_OK;
}
