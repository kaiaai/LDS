// Copyright 2023-2025 KAIA.AI
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
// EXPERIMENTAL - needs testing
// YDLIDAR T-mini Plus / T-mini Pro (ToF LiDAR)
// Uses same protocol as X4 but at 230400 baud
// ToF sensor - no distance correction needed (unlike triangulation)

#include "LDS_YDLIDAR_TMINI.h"

uint32_t LDS_YDLIDAR_TMINI::getSerialBaudRate() {
  return 230400;
}

int LDS_YDLIDAR_TMINI::getSamplingRateHz() {
  return 4000;
}

const char* LDS_YDLIDAR_TMINI::getModelName() {
  return "YDLIDAR T-mini";
}

// Override waitScanDot to use ToF distance calculation (no angle correction)
LDS::result_t LDS_YDLIDAR_TMINI::waitScanDot() {
  uint8_t *packageBuffer = (uint8_t*)&package.package_Head;

  switch(state) {
    case 1:
      goto state1;
    case 2:
      goto state2;
  }

  if (package_Sample_Index == 0) {

    package_Sample_Num = 0;
    package_recvPos = 0;
    recvPos = 0;

    // Read in 10-byte packet header
    while (true) {
state1:
      int currentByte = readSerial();
      if (currentByte < 0) {
        state = 1;
        return ERROR_NOT_READY;
      }

      switch (recvPos) {
        case 0: // 0xAA 1st byte of package header
          if (currentByte != (PH&0xFF)) {
            checkInfo(currentByte);
            continue;
          }
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
          if ((currentByte & 0x01) == CT_RING_START)
            markScanTime();
          break;
        case 3:
          SampleNumlAndCTCal += (currentByte<<RESP_MEAS_ANGLE_SAMPLE_SHIFT);
          package_Sample_Num = currentByte;
          break;
        case 4:
          if (currentByte & RESP_MEAS_CHECKBIT) {
            FirstSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 5:
          FirstSampleAngle += (currentByte<<RESP_MEAS_ANGLE_SAMPLE_SHIFT);
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle>>1;
          break;
        case 6:
          if (currentByte & RESP_MEAS_CHECKBIT) {
            LastSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 7:
          LastSampleAngle += (currentByte<<RESP_MEAS_ANGLE_SAMPLE_SHIFT);
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
          CheckSum += (currentByte<<RESP_MEAS_ANGLE_SAMPLE_SHIFT);
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
      recvPos = 0;
      package_sample_sum = package_Sample_Num<<1;

      // Read in samples, 2 bytes each
      while (true) {
state2:
        int currentByte = readSerial();
        if (currentByte < 0){
          state = 2;
          return ERROR_NOT_READY;
        }

        if ((recvPos & 1) == 1) {
          Valu8Tou16 += (currentByte<<RESP_MEAS_ANGLE_SAMPLE_SHIFT);
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
      state = 0;
      return ERROR_INVALID_PACKET;
    }
    CheckSumCal ^= SampleNumlAndCTCal;
    CheckSumCal ^= LastSampleAngleCal;

    CheckSumResult = CheckSumCal == CheckSum;
  }

  scan_completed = false;
  if (CheckSumResult) {
    scan_completed = (package.package_CT & 0x01) == CT_RING_START;
    if (scan_completed)
      scan_freq_hz = float(package.package_CT >> 1)*0.1f;

    postPacket(packageBuffer, PACKAGE_PAID_BYTES+package_sample_sum, scan_completed);
  }

  // Process the buffered packet - ToF version (no angle correction)
  while(true) {
    node_info_t node;

    node.sync_quality = NODE_DEFAULT_QUALITY;

    if (CheckSumResult == true) {
      // ToF: Distance is direct, no correction needed
      node.distance_q2 = package.packageSampleDistance[package_Sample_Index];

      float sampleAngle = IntervalSampleAngle*package_Sample_Index;
      float angle = FirstSampleAngle + sampleAngle;

      // Normalize angle to 0-23040 range (360 degrees * 64)
      if (angle < 0) {
        angle += 23040;
      } else if (angle > 23040) {
        angle -= 23040;
      }

      node.angle_q6_checkbit = ((uint16_t)angle << RESP_MEAS_ANGLE_SHIFT) + RESP_MEAS_CHECKBIT;
    } else {
      node.angle_q6_checkbit = RESP_MEAS_CHECKBIT;
      node.distance_q2 = 0;
      package_Sample_Index = 0;
      state = 0;
      return ERROR_CHECKSUM;
    }

    // ToF distance: direct mm value (no /4 scaling like triangulation)
    float point_distance_mm = node.distance_q2;
    float point_angle = (node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT)*0.015625f;
    uint8_t point_quality = node.sync_quality;

    postScanPoint(point_angle, point_distance_mm, point_quality, scan_completed);
    scan_completed = false;

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
