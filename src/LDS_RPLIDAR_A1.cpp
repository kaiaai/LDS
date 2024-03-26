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
// Based on https://github.com/robopeak/rplidar_arduino

#include "LDS_RPLIDAR_A1.h"

void LDS_RPLIDAR_A1::init() {
  ring_start_ms[0] = ring_start_ms[1] = 0;
  recvPos = 0;
  enableMotor(false);
}

LDS::result_t LDS_RPLIDAR_A1::start() {
  enableMotor(false);

  abort();

  device_info_t deviceinfo;
  if (getDeviceInfo(deviceinfo, 500) != RESULT_OK)
    return ERROR_DEVICE_INFO;

  postInfo(INFO_MODEL, "RPLIDAR " + String(deviceinfo.model));

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

  return LDS::RESULT_OK;
}

uint32_t LDS_RPLIDAR_A1::getSerialBaudRate() {
  return 115200;
}

float LDS_RPLIDAR_A1::getCurrentScanFreqHz() {
  unsigned long int scan_period_ms = ring_start_ms[0] - ring_start_ms[1];
  return (!motor_enabled || scan_period_ms == 0) ? 0 : 1000.0f/float(scan_period_ms);
}

float LDS_RPLIDAR_A1::getTargetScanFreqHz() {
  return DEFAULT_VALUE;
}

int LDS_RPLIDAR_A1::getSamplingRateHz() {
  return 4000; // TODO high speed mode 8000
}

LDS::result_t LDS_RPLIDAR_A1::stop() {
  if (isActive())
    abort();
  enableMotor(false);
  return RESULT_OK;
}

void LDS_RPLIDAR_A1::enableMotor(bool enable) {
  motor_enabled = enable;

  setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
  setMotorPin(enable ? VALUE_HIGH : VALUE_LOW, LDS_MOTOR_PWM_PIN);
}

bool LDS_RPLIDAR_A1::isActive() {
  return motor_enabled;
}

LDS::result_t LDS_RPLIDAR_A1::setScanTargetFreqHz(float freq) {
  return freq <= 0 ? RESULT_OK : ERROR_NOT_IMPLEMENTED;
}

void LDS_RPLIDAR_A1::markScanTime() {
  ring_start_ms[1] = ring_start_ms[0];
  ring_start_ms[0] = millis();
}

LDS::result_t LDS_RPLIDAR_A1::waitScanDot() {

  while (true) {
    if (recvPos >= sizeof(node_info_t))
      recvPos = 0;

    int current_byte = readSerial();
    if (current_byte < 0)
      return ERROR_NOT_READY;

    switch (recvPos) {
    case 0:
      if (((current_byte >> 1) ^ current_byte) & 0x01)
        break;
      continue;
    case 1:
      if (current_byte & RESP_MEAS_CHECKBIT)
        break;
      recvPos = 0;
      if (((current_byte >> 1) ^ current_byte) & 0x01)
        break;
      continue;
    }

    uint8_t *nodebuf = (uint8_t*)&node;
    nodebuf[recvPos++] = current_byte;

    if (recvPos == sizeof(node_info_t)) {
      float distance_mm = node.distance_q2 * 0.25f;
      float angle_deg = (node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT) * 0.015625f; // /64
      uint8_t quality = (node.sync_quality >> RESP_MEAS_QUALITY_SHIFT);
      bool scan_completed = (node.sync_quality & RESP_MEAS_SYNCBIT);

      if (scan_completed)
        markScanTime();
      postPacket(nodebuf, sizeof(node_info_t), scan_completed);
      postScanPoint(angle_deg, distance_mm, quality, scan_completed);

      recvPos = 0;
      return RESULT_OK;
    }
  }
}

void LDS_RPLIDAR_A1::loop() {
  while (true) {
    result_t result = waitScanDot();
    if (result == ERROR_NOT_READY)
      return;
    if (result < RESULT_OK)
      postError(result, "");
  }
}

LDS::result_t LDS_RPLIDAR_A1::abort() {
  return sendCommand(CMD_STOP, NULL, 0);
}

LDS::result_t LDS_RPLIDAR_A1::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
  cmd_packet_t pkt_header;
  cmd_packet_t * header = &pkt_header;
  uint8_t checksum = 0;

  if (payloadsize && payload)
    cmd |= CMDFLAG_HAS_PAYLOAD;

  header->syncByte = CMD_SYNC_BYTE;
  header->cmd_flag = cmd & 0xff;

  writeSerial((uint8_t *)header, 2);
  if ((cmd & CMDFLAG_HAS_PAYLOAD)) {
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

LDS::result_t LDS_RPLIDAR_A1::getDeviceInfo(device_info_t & info, uint32_t timeout) {
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

LDS::result_t LDS_RPLIDAR_A1::waitResponseHeader(ans_header_t * header, uint32_t timeout) {
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

LDS::result_t LDS_RPLIDAR_A1::getHealth(device_health_t & health, uint32_t timeout) {
  uint8_t recvPos = 0;
  uint32_t currentTs = millis();
  uint8_t *infobuf = (uint8_t*)&health;

  LDS::result_t ans = sendCommand(CMD_GET_DEV_HEALTH, NULL, 0);
  if (ans != RESULT_OK)
    return ans;

  ans_header_t response_header;
  ans = waitResponseHeader(&response_header, timeout);
  if (ans != RESULT_OK)
    return ans;

  if (response_header.type != ANS_TYPE_DEV_HEALTH)
    return ERROR_INVALID_DATA;

  if (response_header.size < sizeof(device_health_t))
    return ERROR_INVALID_PACKET;
  
  while ((millis() - currentTs) <= timeout) {
    int current_byte = readSerial();
    if (current_byte < 0)
      continue;
    
    infobuf[recvPos++] = current_byte;

    if (recvPos == sizeof(device_health_t))
      return RESULT_OK;
  }

  return ERROR_TIMEOUT;
}

LDS::result_t LDS_RPLIDAR_A1::startScan(bool force, uint32_t timeout ) {
  abort();

  LDS::result_t ans = sendCommand(force ? CMD_FORCE_SCAN : CMD_SCAN, NULL, 0);
  if (ans != RESULT_OK)
    return ans;

  ans_header_t response_header;
  ans = waitResponseHeader(&response_header, timeout);
  if (ans != RESULT_OK)
    return ans;

  if (response_header.type != ANS_TYPE_MEAS)
    return ERROR_INVALID_DATA;

  if (response_header.size < sizeof(node_info_t))
    return ERROR_INVALID_PACKET;

  return RESULT_OK;
}

const char* LDS_RPLIDAR_A1::getModelName() { return "RPLIDAR A1"; }