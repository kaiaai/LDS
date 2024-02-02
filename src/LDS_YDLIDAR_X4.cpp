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
//   Copyright 2015 - 2018 EAI TEAM http://www.eaibot.com
//   https://github.com/EAIBOT/ydlidar_arduino

#include "LDS_YDLIDAR_X4.h"

void LDS_YDLIDAR_X4::init() {
  target_scan_freq = sampling_rate = ERROR_UNKNOWN;
  ring_start_ms[0] = ring_start_ms[1] = 0;
  scan_freq = 0;
  scan_completed = false;
  enableMotor(false);
}

LDS::result_t LDS_YDLIDAR_X4::start() {
  // Initialize
  enableMotor(false);

  abort();

  device_info deviceinfo;
  if (getDeviceInfo(deviceinfo, 500) != LDS::RESULT_OK)
    return ERROR_DEVICE_INFO;

  String model = "YDLIDAR ";
  switch (deviceinfo.model) {
    case 1:
      model += "F4";
      sampling_rate = 4000;
      target_scan_freq = 7;
      break;
    case 4:
      model += "S4";
      sampling_rate = 4000;
      target_scan_freq = 7;
      break;
    case 5:
      model += "G4";
      sampling_rate = 9000;
      target_scan_freq = 7;
      break;
    case 6:
      model += "X4";
      sampling_rate = 5000;
      target_scan_freq = 7;
      break;
    default:
      model = "Unknown";
      sampling_rate = ERROR_UNKNOWN;
      target_scan_freq = ERROR_UNKNOWN;
  }
  postInfo(INFO_MODEL, model);
  postInfo(INFO_SAMPLING_RATE, String(sampling_rate));
  postInfo(INFO_DEFAULT_TARGET_SCAN_FREQ_HZ, String(target_scan_freq));

  uint16_t maxv = (uint16_t)(deviceinfo.firmware_version >> 8);
  uint16_t midv = (uint16_t)(deviceinfo.firmware_version & 0xff) / 10;
  uint16_t minv = (uint16_t)(deviceinfo.firmware_version & 0xff) % 10;
  if (midv == 0) {
    midv = minv;
    minv = 0;
  }

  postInfo(INFO_FIRMWARE_VERSION, String(maxv + '.' + midv + '.' + minv));
  postInfo(INFO_HARDWARE_VERSION, String((uint16_t)deviceinfo.hardware_version));

  String serial_num;
  for (int i = 0; i < 16; i++)
    serial_num += String(deviceinfo.serialnum[i] & 0xff, HEX);
  postInfo(INFO_SERIAL_NUMBER, serial_num);
  delay(100);

  device_health healthinfo;
  if (getHealth(healthinfo, 100) != LDS::RESULT_OK)    
    return ERROR_DEVICE_HEALTH;
  postInfo(INFO_DEVICE_HEALTH, healthinfo.status == 0 ? "OK" : "bad");

  // Start
  if (startScan() != LDS::RESULT_OK)
    return ERROR_START_SCAN;
  enableMotor(true);
  delay(1000);

  return LDS::RESULT_OK;
}

uint32_t LDS_YDLIDAR_X4::getSerialBaudRate() {
  return 128000;
}

float LDS_YDLIDAR_X4::getCurrentScanFreqHz() {
  unsigned long int scan_period_ms = ring_start_ms[0] - ring_start_ms[1];
  return (!motor_enabled || scan_period_ms == 0) ? 0 : 1000.0f/float(scan_period_ms);
}

float LDS_YDLIDAR_X4::getTargetScanFreqHz() {
  return target_scan_freq;
}

int LDS_YDLIDAR_X4::getSamplingRateHz() {
  return sampling_rate;
}

void LDS_YDLIDAR_X4::stop() {
  if (isActive())
    abort();
  enableMotor(false);
}

void LDS_YDLIDAR_X4::enableMotor(bool enable) {
  motor_enabled = enable;

  setMotorPin(DIR_INPUT, LDS_MOTOR_PWM_PIN);
  setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_EN_PIN);

  // Entire LDS on/off
  setMotorPin(enable ? VALUE_HIGH : VALUE_LOW, LDS_MOTOR_EN_PIN);
}

bool LDS_YDLIDAR_X4::isActive() {
  return motor_enabled;
}

LDS::result_t LDS_YDLIDAR_X4::setScanTargetFreqHz(float freq) {
  return ERROR_NOT_IMPLEMENTED;
}

LDS::result_t LDS_YDLIDAR_X4::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  return ERROR_NOT_IMPLEMENTED;
}

LDS::result_t LDS_YDLIDAR_X4::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  return ERROR_NOT_IMPLEMENTED;
}

void LDS_YDLIDAR_X4::markScanTime() {
  ring_start_ms[1] = ring_start_ms[0];
  ring_start_ms[0] = millis();
}

LDS::result_t LDS_YDLIDAR_X4::waitScanDot() {

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
          if ((currentByte & 0x01) == CT_RING_START)
            markScanTime();
          //if ((currentByte != CT_NORMAL) && (currentByte != CT_RING_START)) { 
          //  recvPos = 0;
          //  continue;
          //}
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

    CheckSumResult = CheckSumCal == CheckSum;
    //if (CheckSumCal != CheckSum){  
    //  CheckSumResult = false;
    //} else {
    //  CheckSumResult = true;
    //}
  }

  //if (CheckSumResult)
  //  postPacket(packageBuffer, PACKAGE_PAID_BYTES+package_sample_sum,
  //    package.package_CT == CT_RING_START);

  scan_completed = false;
  if (CheckSumResult) {
    scan_completed = (package.package_CT & 0x01) == CT_RING_START;
    if (scan_completed)
      scan_freq = package.package_CT >> 1;
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

    //postScanPoint(point_angle, point_distance_mm, point_quality, point_startBit);
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

void LDS_YDLIDAR_X4::loop() {
  result_t result = waitScanDot();
  if (result < RESULT_OK)
    postError(result, "waitScanDot()");
}

LDS::result_t LDS_YDLIDAR_X4::abort() {
  // stop the scanPoint operation
  return sendCommand(LIDAR_CMD_FORCE_STOP, NULL, 0);
}

LDS::result_t LDS_YDLIDAR_X4::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
  //send data to serial
  cmd_packet pkt_header;
  cmd_packet * header = &pkt_header;
  uint8_t checksum = 0;

  if (payloadsize && payload)
    cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;

  header->syncByte = LIDAR_CMD_SYNC_BYTE;
  header->cmd_flag = cmd&0xff;

  writeSerial((uint8_t *)header, 2) ;
  if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)) {
    checksum ^= LIDAR_CMD_SYNC_BYTE;
    checksum ^= (cmd&0xff);
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos)
      checksum ^= ((uint8_t *)payload)[pos];

    uint8_t sizebyte = payloadsize;
    writeSerial(&sizebyte, 1);
    writeSerial((const uint8_t *)payload, sizebyte);
    writeSerial(&checksum, 1);
  }
  return LDS::RESULT_OK;
}

LDS::result_t LDS_YDLIDAR_X4::getDeviceInfo(device_info & info, uint32_t timeout) {
  LDS::result_t ans;
  uint8_t  recvPos = 0;
  uint32_t currentTs = millis();
  uint32_t remainingtime;
  uint8_t *infobuf = (uint8_t*)&info;
  lidar_ans_header response_header;

  ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO, NULL, 0);
  if (ans != LDS::RESULT_OK)
    return ans;

  if ((ans = waitResponseHeader(&response_header, timeout)) != LDS::RESULT_OK)
    return ans;

  if (response_header.type != LIDAR_ANS_TYPE_DEVINFO)
    return ERROR_INVALID_PACKET;

  if (response_header.size < sizeof(lidar_ans_header))
    return ERROR_INVALID_PACKET;

  while ((remainingtime=millis() - currentTs) <= timeout) {
    int currentbyte = readSerial();
    if (currentbyte<0)
      continue;    
    infobuf[recvPos++] = currentbyte;

    if (recvPos == sizeof(device_info))
      return LDS::RESULT_OK;
  }

  return ERROR_TIMEOUT;
}

LDS::result_t LDS_YDLIDAR_X4::waitResponseHeader(lidar_ans_header * header, uint32_t timeout) {
  // wait response header
  int recvPos = 0;
  uint32_t startTs = millis();
  uint8_t *headerBuffer = (uint8_t *)(header);
  uint32_t waitTime;

  while ((waitTime=millis() - startTs) <= timeout) {
    int currentbyte = readSerial();
    if (currentbyte<0)
      continue;

    switch (recvPos) {
      case 0:
        if (currentbyte != LIDAR_ANS_SYNC_BYTE1)
          continue;
        break;
      case 1:
        if (currentbyte != LIDAR_ANS_SYNC_BYTE2) {
            recvPos = 0;
            continue;
        }
        break;
    }
    headerBuffer[recvPos++] = currentbyte;

    if (recvPos == sizeof(lidar_ans_header))
      return LDS::RESULT_OK;
  }
  return ERROR_TIMEOUT;
}

// ask the YDLIDAR for its device health
LDS::result_t LDS_YDLIDAR_X4::getHealth(device_health & health, uint32_t timeout) {
  LDS::result_t ans;
  uint8_t recvPos = 0;
  uint32_t currentTs = millis();
  uint32_t remainingtime;
  uint8_t *infobuf = (uint8_t*)&health;
  lidar_ans_header response_header;

  ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0);
  if (ans != LDS::RESULT_OK)
    return ans;

  if ((ans = waitResponseHeader(&response_header, timeout)) != LDS::RESULT_OK)
    return ans;

  if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH)
    return ERROR_INVALID_PACKET;

  if (response_header.size < sizeof(device_health))
    return ERROR_INVALID_PACKET;

  while ((remainingtime=millis() - currentTs) <= timeout) {
    int currentbyte = readSerial();

    if (currentbyte < 0)
      continue;
    infobuf[recvPos++] = currentbyte;

    if (recvPos == sizeof(device_health))
      return LDS::RESULT_OK;
  }
  return ERROR_TIMEOUT;
}

LDS::result_t LDS_YDLIDAR_X4::startScan(bool force, uint32_t timeout ) {
  // start the scanPoint operation
  LDS::result_t ans;

  abort(); //force the previous operation to stop

  if ((ans = sendCommand(force ? LIDAR_CMD_FORCE_SCAN : LIDAR_CMD_SCAN, NULL, 0)) != LDS::RESULT_OK)
    return ans;

  lidar_ans_header response_header;
  if ((ans = waitResponseHeader(&response_header, timeout)) != LDS::RESULT_OK)
    return ans;

  if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT)
    return ERROR_INVALID_PACKET;

  if (response_header.size < sizeof(node_info))
    return ERROR_INVALID_PACKET;

  return LDS::RESULT_OK;
}
