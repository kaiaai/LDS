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
// Hitachi-LG HLS-LFCD2 / LDS-01 (TurtleBot3 LiDAR)
// Based on https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver
//
// Note: LiDAR RX pin is 3.3V tolerant only!
// Use voltage divider if connecting to 5V Arduino

#include "LDS_HLS_LFCD2.h"

void LDS_HLS_LFCD2::init() {
  motor_enabled = false;
  motor_rpm = 0;
  parser_idx = 0;
  packet_index = 0;
  enableMotor(false);
}

LDS::result_t LDS_HLS_LFCD2::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return RESULT_OK;
}

uint32_t LDS_HLS_LFCD2::getSerialBaudRate() {
  return 230400;
}

float LDS_HLS_LFCD2::getCurrentScanFreqHz() {
  return motor_rpm / 60.0f;
}

float LDS_HLS_LFCD2::getTargetScanFreqHz() {
  return 5.0f;  // ~300 RPM nominal
}

int LDS_HLS_LFCD2::getSamplingRateHz() {
  return 1800;  // 360 points * 5 Hz
}

LDS::result_t LDS_HLS_LFCD2::stop() {
  enableMotor(false);
  return RESULT_OK;
}

void LDS_HLS_LFCD2::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    // Send 'b' to start scanning
    uint8_t cmd = 'b';
    writeSerial(&cmd, 1);
  } else {
    // Send 'e' to stop scanning
    uint8_t cmd = 'e';
    writeSerial(&cmd, 1);
  }
}

bool LDS_HLS_LFCD2::isActive() {
  return motor_enabled;
}

LDS::result_t LDS_HLS_LFCD2::setScanTargetFreqHz(float freq) {
  // HLS-LFCD2 doesn't support speed control
  return freq <= 0 ? RESULT_OK : ERROR_NOT_IMPLEMENTED;
}

void LDS_HLS_LFCD2::loop() {
  while (true) {
    int c = readSerial();
    if (c < 0)
      break;

    result_t result = processByte((uint8_t)c);
    if (result < 0)
      postError(result, "");
  }
}

uint16_t LDS_HLS_LFCD2::calcChecksum(uint8_t* data, uint16_t len) {
  uint32_t chk32 = 0;
  for (uint16_t i = 0; i < len / 2; i++) {
    uint16_t word = data[i * 2] | (data[i * 2 + 1] << 8);
    chk32 = (chk32 << 1) + word;
  }
  uint32_t checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  return checksum & 0x7FFF;
}

LDS::result_t LDS_HLS_LFCD2::processByte(uint8_t c) {
  LDS::result_t result = RESULT_OK;
  uint8_t* rx_buffer = (uint8_t*)&packet;

  if (parser_idx >= PACKET_SIZE) {
    parser_idx = 0;
    return RESULT_OK;
  }

  rx_buffer[parser_idx++] = c;

  switch (parser_idx) {
  case 1:
    if (c != START_BYTE) {
      parser_idx = 0;
    }
    break;

  case 2:
    // Index byte: 0xA0 + packet_number (0-59)
    if (c < 0xA0 || c > 0xA0 + PACKETS_PER_SCAN - 1) {
      parser_idx = 0;
    } else {
      packet_index = c - 0xA0;
    }
    break;

  case PACKET_SIZE:
    // Complete packet received
    {
      // Verify checksum
      uint16_t received_checksum = packet.checksum;
      uint16_t calc_checksum = calcChecksum(rx_buffer, PACKET_SIZE - 2);

      if (received_checksum != calc_checksum) {
        result = ERROR_CHECKSUM;
      } else {
        motor_rpm = packet.rpm;
        bool scan_completed = (packet_index == 0);

        postPacket(rx_buffer, PACKET_SIZE, scan_completed);

        // Process 6 readings in this packet
        for (uint8_t i = 0; i < READINGS_PER_PACKET; i++) {
          uint16_t point_index = packet_index * READINGS_PER_PACKET + i;
          // Angle: 360 degrees / 360 points = 1 degree per point
          // HLS-LFCD outputs in reverse order
          float angle_deg = 359.0f - point_index;
          if (angle_deg < 0)
            angle_deg += 360.0f;

          float distance_mm = packet.readings[i].range_mm;
          float quality = packet.readings[i].intensity;

          bool point_scan_completed = scan_completed && (i == 0);
          postScanPoint(angle_deg, distance_mm, quality, point_scan_completed);
        }
      }
    }
    parser_idx = 0;
    break;
  }

  if (result < RESULT_OK)
    parser_idx = 0;

  return result;
}

const char* LDS_HLS_LFCD2::getModelName() {
  return "Hitachi-LG HLS-LFCD2";
}
