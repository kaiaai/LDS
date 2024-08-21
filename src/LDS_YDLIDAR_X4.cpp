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
// Based on https://github.com/EAIBOT/ydlidar_arduino

#include "LDS_YDLIDAR_X4.h"

void LDS_YDLIDAR_X4::init() {
  ring_start_ms[0] = ring_start_ms[1] = 0;
  scan_freq_hz = 0;
  scan_completed = false;
  enableMotor(false);
}

LDS::result_t LDS_YDLIDAR_X4::start() {
  enableMotor(false);

  abort();

  device_info_t deviceinfo;
  if (getDeviceInfo(deviceinfo, 500) != RESULT_OK)
    return ERROR_DEVICE_INFO;

  if (deviceinfo.model == 6) {
    postInfo(INFO_MODEL, getModelName());
  } else {
    postError(ERROR_INVALID_MODEL, String(deviceinfo.model));
  }

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

  device_health_t healthinfo;
  if (getHealth(healthinfo, 100) != RESULT_OK)    
    return ERROR_DEVICE_HEALTH;
  postInfo(INFO_DEVICE_HEALTH, healthinfo.status == 0 ? "OK" : "bad");

  // Start
  if (startScan() != RESULT_OK)
    return ERROR_START_SCAN;
  enableMotor(true);
  delay(1000);

  return RESULT_OK;
}

uint32_t LDS_YDLIDAR_X4::getSerialBaudRate() {
  return 128000;
}

float LDS_YDLIDAR_X4::getCurrentScanFreqHz() {
  unsigned long int scan_period_ms = ring_start_ms[0] - ring_start_ms[1];
  return (!motor_enabled || scan_period_ms == 0) ? 0 : 1000.0f/float(scan_period_ms);
}

float LDS_YDLIDAR_X4::getTargetScanFreqHz() {
  return DEFAULT_VALUE;
}

int LDS_YDLIDAR_X4::getSamplingRateHz() {
  return 5000;
}

LDS::result_t LDS_YDLIDAR_X4::stop() {
  if (isActive())
    abort();
  enableMotor(false);
  return RESULT_OK;
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
  return freq <= 0 ? RESULT_OK : ERROR_NOT_IMPLEMENTED;
}

void LDS_YDLIDAR_X4::markScanTime() {
  ring_start_ms[1] = ring_start_ms[0];
  ring_start_ms[0] = millis();
}

void LDS_YDLIDAR_X4::checkInfo(int currentByte) {
  static String s;
  static uint8_t state = 0;
  const uint8_t header[] = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};

  switch (state) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
      if (currentByte == header[state]) {
         state++;
      } else {
        state = 0;
        s = "";
      }
      break;
    case 27:
      postInfo(INFO_MODEL, s);
      state = 0;
      s = "";
      break;
    case 7:
      s = "Model 0x";
      if (currentByte < 16)
        s = s + '0';
      s = s + String(currentByte, HEX);
      state++;
      break;
    case 8:
      s = s + ", firmware v" + String(currentByte);
      state++;
      break;
    case 9:
      s = s + '.' + String(currentByte);
      state++;
      break;
    case 10:
      s = s + ", hardware v" + String(currentByte);
      state++;
      break;
    case 11:
      s = s + ", S/N " + String(currentByte);
      state++;
      break;
    default:
      s = s + String(currentByte);
      state++;
      break;
  }
}

LDS::result_t LDS_YDLIDAR_X4::waitScanDot() {
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
          //if ((currentByte != CT_NORMAL) && (currentByte != CT_RING_START)) { 
          //  recvPos = 0;
          //  continue;
          //}
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
      // Packet header looks valid size-wise
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
    scan_completed = (package.package_CT & 0x01) == CT_RING_START;
    if (scan_completed)
      //scan_freq = package.package_CT >> 1;
      //F = CT[bit(7:1)]/10 (when CT[bit(7:1)] = 1).
      scan_freq_hz = float(package.package_CT >> 1)*0.1f;

    postPacket(packageBuffer, PACKAGE_PAID_BYTES+package_sample_sum, scan_completed);
  }

  // Process the buffered packet
  while(true) {
    node_info_t node;
  
    node.sync_quality = NODE_DEFAULT_QUALITY;
  
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
          + AngleCorrectForDistance + 23040))<<RESP_MEAS_ANGLE_SHIFT)
          + RESP_MEAS_CHECKBIT;
      } else {
        if ((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040) {
          node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle
            + AngleCorrectForDistance - 23040))<<RESP_MEAS_ANGLE_SHIFT)
            + RESP_MEAS_CHECKBIT;
        } else {
          node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle
            + AngleCorrectForDistance))<<RESP_MEAS_ANGLE_SHIFT)
            + RESP_MEAS_CHECKBIT;
        } 
      }
    } else {
      // Invalid checksum
      //node.sync_quality = NODE_DEFAULT_QUALITY + NODE_NOT_SYNC;
      node.angle_q6_checkbit = RESP_MEAS_CHECKBIT;
      node.distance_q2 = 0;
      package_Sample_Index = 0;
      state = 0;
      return ERROR_CHECKSUM;
    }
  
    // Dump out processed data
    float point_distance_mm = node.distance_q2*0.25f;
    float point_angle = (node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT)*0.015625f; // /64.0f
    //uint8_t point_quality = (node.sync_quality>>RESP_MEAS_QUALITY_SHIFT);
    uint8_t point_quality = node.sync_quality;
    //bool point_startBit = (node.sync_quality & RESP_MEAS_SYNCBIT);

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
  while (true) {
    result_t result = waitScanDot();
    if (result == ERROR_NOT_READY)
      break;
    if (result < RESULT_OK)
      postError(result, "");
  }
}

LDS::result_t LDS_YDLIDAR_X4::abort() {
  return sendCommand(CMD_FORCE_STOP, NULL, 0);
}

LDS::result_t LDS_YDLIDAR_X4::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
  cmd_packet_t pkt_header;
  cmd_packet_t * header = &pkt_header;
  uint8_t checksum = 0;

  if (payloadsize && payload)
    cmd |= CMDFLAG_HAS_PAYLOAD;

  header->syncByte = CMD_SYNC_BYTE;
  header->cmd_flag = cmd & 0xff;

  writeSerial((uint8_t *)header, 2) ;
  if (cmd & CMDFLAG_HAS_PAYLOAD) {
    checksum ^= CMD_SYNC_BYTE;
    checksum ^= (cmd & 0xff);
    checksum ^= (payloadsize & 0xFF);

    for (size_t pos = 0; pos < payloadsize; ++pos)
      checksum ^= ((uint8_t *)payload)[pos];

    uint8_t sizebyte = payloadsize;
    writeSerial(&sizebyte, 1);
    writeSerial((const uint8_t *)payload, sizebyte);
    writeSerial(&checksum, 1);
  }
  return RESULT_OK;
}

LDS::result_t LDS_YDLIDAR_X4::getDeviceInfo(device_info_t & info, uint32_t timeout) {
  uint8_t recvPos = 0;
  uint32_t currentTs = millis();
  uint8_t *infobuf = (uint8_t*)&info;

  LDS::result_t ans = sendCommand(CMD_GET_DEV_INFO, NULL, 0);
  if (ans != RESULT_OK)
    return ans;

  ans_header_t response_header;
  ans = waitResponseHeader(&response_header, timeout);
  if (ans != RESULT_OK)
    return ans;

  if (response_header.type != ANS_TYPE_DEV_INFO)
    return ERROR_INVALID_PACKET;

  if (response_header.size < sizeof(ans_header_t))
    return ERROR_INVALID_PACKET;

  while ((millis() - currentTs) <= timeout) {
    int current_byte = readSerial();
    if (current_byte < 0)
      continue;    
    infobuf[recvPos++] = current_byte;

    if (recvPos == sizeof(device_info_t))
      return RESULT_OK;
  }

  return ERROR_TIMEOUT;
}

LDS::result_t LDS_YDLIDAR_X4::waitResponseHeader(ans_header_t * header, uint32_t timeout) {
  int recvPos = 0;
  uint32_t startTs = millis();
  uint8_t *headerBuffer = (uint8_t *)header;

  while ((millis() - startTs) <= timeout) {
    int current_byte = readSerial();
    if (current_byte < 0)
      continue;

    switch (recvPos) {
      case 0:
        if (current_byte != ANS_SYNC_BYTE1)
          continue;
        break;
      case 1:
        if (current_byte != ANS_SYNC_BYTE2) {
            recvPos = 0;
            continue;
        }
        break;
    }
    headerBuffer[recvPos++] = current_byte;

    if (recvPos == sizeof(ans_header_t))
      return RESULT_OK;
  }
  return ERROR_TIMEOUT;
}

LDS::result_t LDS_YDLIDAR_X4::getHealth(device_health_t & health, uint32_t timeout) {
  uint8_t recvPos = 0;
  uint32_t currentTs = millis();
  uint8_t *infobuf = (uint8_t*)&health;

  LDS::result_t ans = sendCommand(CMD_GET_DEV_HEALTH, NULL, 0);
  if (ans != RESULT_OK)
    return ans;

  ans_header_t response_header;
  ans = waitResponseHeader(&response_header, timeout);
  if (ans != LDS::RESULT_OK)
    return ans;

  if (response_header.type != ANS_TYPE_DEV_HEALTH)
    return ERROR_INVALID_PACKET;

  if (response_header.size < sizeof(device_health_t))
    return ERROR_INVALID_PACKET;

  while ((millis() - currentTs) <= timeout) {
    int currentbyte = readSerial();

    if (currentbyte < 0)
      continue;
    infobuf[recvPos++] = currentbyte;

    if (recvPos == sizeof(device_health_t))
      return RESULT_OK;
  }
  return ERROR_TIMEOUT;
}

LDS::result_t LDS_YDLIDAR_X4::startScan(bool force, uint32_t timeout ) {
  abort();

  LDS::result_t ans = sendCommand(force ? CMD_FORCE_SCAN : CMD_SCAN, NULL, 0);
  if (ans != RESULT_OK)
    return ans;

  ans_header_t response_header;
  ans = waitResponseHeader(&response_header, timeout);
  if (ans != RESULT_OK)
    return ans;

  if (response_header.type != ANS_TYPE_MEAS)
    return ERROR_INVALID_PACKET;

  if (response_header.size < sizeof(node_info_t))
    return ERROR_INVALID_PACKET;

  return RESULT_OK;
}

const char* LDS_YDLIDAR_X4::getModelName() { return "YDLIDAR X4"; }