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
// Based on https://github.com/YDLIDAR/lidarCar/

#include "LDS_YDLIDAR_SCL.h"

const char* LDS_YDLIDAR_SCL::getModelName() { return "YDLIDAR SCL"; }

void LDS_YDLIDAR_SCL::init() {
  LDS_YDLIDAR_X4::init();

  pwm_val = PWM_START;
  scan_freq_setpoint_hz = DEFAULT_SCAN_FREQ_HZ;

  scanFreqPID.init(&scan_freq_hz, &pwm_val, &scan_freq_setpoint_hz, 1.0e-2, 2.5e-2, 0.0, PID_v1::DIRECT);
  scanFreqPID.SetOutputLimits(0, 1.0);
  scanFreqPID.SetSampleTime(20);
  scanFreqPID.SetMode(PID_v1::AUTOMATIC);
}

LDS::result_t LDS_YDLIDAR_SCL::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return LDS::RESULT_OK;
}

uint32_t LDS_YDLIDAR_SCL::getSerialBaudRate() {
  return 115200;
}

int LDS_YDLIDAR_SCL::getSamplingRateHz() {
  return 3000;
}

float LDS_YDLIDAR_SCL::getCurrentScanFreqHz() {
  return motor_enabled ? scan_freq_hz : 0;
}

LDS::result_t LDS_YDLIDAR_SCL::stop() {
  enableMotor(false);
  return RESULT_OK;
}

void LDS_YDLIDAR_SCL::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    //setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    //setMotorPin(VALUE_HIGH, LDS_MOTOR_PWM_PIN);
    setMotorPin(DIR_OUTPUT_PWM, LDS_MOTOR_PWM_PIN);
    setMotorPin(pwm_val, LDS_MOTOR_PWM_PIN);
  } else {
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }
}

void LDS_YDLIDAR_SCL::loop() {
  LDS_YDLIDAR_X4::loop();

  if (motor_enabled) {
    scanFreqPID.Compute();

    if (pwm_val != pwm_last) {
      setMotorPin(pwm_val, LDS_MOTOR_PWM_PIN);
      pwm_last = pwm_val;
    }
  }
}

LDS::result_t LDS_YDLIDAR_SCL::setScanTargetFreqHz(float freq) {
  scan_freq_setpoint_hz = freq <= 0 ? DEFAULT_SCAN_FREQ_HZ : freq;
  return RESULT_OK;
}

float LDS_YDLIDAR_SCL::getTargetScanFreqHz() {
  return scan_freq_setpoint_hz;
}

LDS::result_t LDS_YDLIDAR_SCL::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  scanFreqPID.SetTunings(Kp, Ki, Kd);
  return RESULT_OK;
}

LDS::result_t LDS_YDLIDAR_SCL::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  scanFreqPID.SetSampleTime(sample_period_ms);
  return RESULT_OK;
}

LDS::result_t LDS_YDLIDAR_SCL::waitScanDot() {
  uint8_t *packageBuffer = (uint8_t*)&package_scl.package_Head;

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
      int currentByte = readSerial();
      if (currentByte < 0) {
        state = 1;
        return ERROR_NOT_READY;
      }

      switch (recvPos) {
        case 0: // 0xAA 1st byte of package header
          if (currentByte != 0xAA) { // (PH&0xFF)
            checkInfo(currentByte);
            continue;
          }
          break;
        case 1: // 0x55 2nd byte of package header
          CheckSumCal = 0x55AA; // PH
          if (currentByte != 0x55) { // (PH>>8)
            recvPos = 0;
            continue;
          }
          break;
        case 2:
          SampleNumlAndCTCal = currentByte;
          if ((currentByte & 0x01) == 0x01) // CT_RING_START
            markScanTime();
          break;
        case 3:
          SampleNumlAndCTCal += (currentByte << 8);
          package_Sample_Num = currentByte;
          break;
        case 4:
          if (currentByte & 0x01) {
            FirstSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 5:
          FirstSampleAngle += (currentByte << 8);
          CheckSumCal ^= FirstSampleAngle;
          FirstSampleAngle = FirstSampleAngle>>1; // degrees*64
          break;
        case 6:
          if (currentByte & 0x01) {
            LastSampleAngle = currentByte;
          } else {
            recvPos = 0;
            continue;
          }
          break;
        case 7:
          LastSampleAngle += (currentByte << 8);
          LastSampleAngleCal = LastSampleAngle;
          LastSampleAngle = LastSampleAngle >> 1;
          if (package_Sample_Num == 1) {
            IntervalSampleAngle = 0;
          } else {
            if (LastSampleAngle < FirstSampleAngle) {
              if ((FirstSampleAngle > 270*64) && (LastSampleAngle < 90*64)) {
                IntervalSampleAngle = ((float)(360*64 + LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
                IntervalSampleAngle_LastPackage = IntervalSampleAngle;
              } else {
                IntervalSampleAngle = IntervalSampleAngle_LastPackage;
              }
            } else {
              IntervalSampleAngle = ((float)(LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
              IntervalSampleAngle_LastPackage = IntervalSampleAngle;
            }
          }
          break;
        case 8:
          CheckSum = currentByte; 
          break;
        case 9:
          CheckSum += (currentByte << 8);
          break;
      }
      packageBuffer[recvPos++] = currentByte;

      if (recvPos == PACKAGE_PAID_BYTES) {
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
      package_sample_sum = package_Sample_Num * 3;

      // Read in samples, 3 bytes each
      while (true) {
state2:
        int currentByte = readSerial();
        if (currentByte < 0){
          state = 2;
          return ERROR_NOT_READY;
        }

        switch (recvPos % 3) {
          case 0:
            CheckSumCal ^= currentByte;
            break;
          case 1:
            Valu8Tou16 = currentByte;
            break;
          case 2:
            Valu8Tou16 += (currentByte << 8);
            CheckSumCal ^= Valu8Tou16;
        }

        packageBuffer[package_recvPos + recvPos] = currentByte;          
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

    CheckSumResult = CheckSumCal == CheckSum;
  }

  scan_completed = false;
  if (CheckSumResult) {
    scan_completed = (package_scl.package_CT & 0x01) == 0x01;
    if (scan_completed)
      scan_freq_hz = float(package_scl.package_CT >> 1)*0.1f;

    postPacket(packageBuffer, PACKAGE_PAID_BYTES+package_sample_sum, scan_completed);
  }

  // Process the buffered packet
  while(true) {
    node_info_scl_t node;
    cloud_point_scl_t point;
  
    if (CheckSumResult == true) {
      int32_t AngleCorrectForDistance = 0;
      point = package_scl.packageSampleDistance[package_Sample_Index];
      node.distance_mm = (point.distance_lsb >> 2) + (point.distance_msb << 6);
      node.intensity = point.intensity;
      node.quality_flag = point.distance_lsb && 0x03;

      if (node.distance_mm != 0) {
        AngleCorrectForDistance = (int32_t)(atan(17.8f/((float)node.distance_mm))*3666.929888837269f); // *64*360/2/pi
      }

      float sampleAngle = IntervalSampleAngle * package_Sample_Index; // *64, before correction

      node.angle_deg = (FirstSampleAngle + sampleAngle + AngleCorrectForDistance) * 0.015625f; // /64
      if (node.angle_deg < 0) {
        node.angle_deg = node.angle_deg + 360;
      } else {
        if (node.angle_deg > 360) {
          node.angle_deg = node.angle_deg - 360;
        }
      }
    } else {
      // Invalid checksum
      node.angle_deg = 0;
      node.distance_mm = 0;
      package_Sample_Index = 0;
      state = 0;
      return ERROR_CHECKSUM;
    }
  
    // Dump out processed data
    postScanPoint(node.angle_deg, node.distance_mm, node.quality_flag, scan_completed);
    scan_completed = false;

    // Dump finished?
    package_Sample_Index++;
    uint8_t nowPackageNum = package_scl.nowPackageNum;  
    if (package_Sample_Index >= nowPackageNum) {
      package_Sample_Index = 0;
      break;
    }
  }
  state = 0;
  return LDS::RESULT_OK;
}