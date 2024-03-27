// Copyright 2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
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

#include "LDS_CAMSENSE_X1.h"

void LDS_CAMSENSE_X1::init() {
  rotation_speed = 0;
  parser_idx = 0;
  //start_angle_prev = 0;
  end_angle_prev = 0;
}

LDS::result_t LDS_CAMSENSE_X1::start() {
  postInfo(INFO_MODEL, getModelName());
  return RESULT_OK;
}

uint32_t LDS_CAMSENSE_X1::getSerialBaudRate() {
  return 115200;
}

float LDS_CAMSENSE_X1::getTargetScanFreqHz() {
  return 5.2f;
}

int LDS_CAMSENSE_X1::getSamplingRateHz() {
  return 2080;
}

float LDS_CAMSENSE_X1::getCurrentScanFreqHz() {
  static float constexpr ONE_OVER_3840 = 1.0 / 3840;
  return rotation_speed * ONE_OVER_3840;
}

LDS::result_t LDS_CAMSENSE_X1::stop() {
  //enableMotor(false);
  return ERROR_NOT_IMPLEMENTED;
}

bool LDS_CAMSENSE_X1::isActive() {
  return true; // motor runs always
}

LDS::result_t LDS_CAMSENSE_X1::setScanTargetFreqHz(float freq) {
  return ERROR_NOT_IMPLEMENTED;
}

void LDS_CAMSENSE_X1::loop() {
  while (true) {
    int c = readSerial();
    if (c < 0)
      break;

    result_t result = processByte((uint8_t)c);
    if (result < 0)
      postError(result, "");
  }
}

LDS::result_t LDS_CAMSENSE_X1::processByte(uint8_t c) {
  LDS::result_t result = RESULT_OK;
  uint8_t * rx_buffer = (uint8_t *)&scan_packet;

  if (parser_idx >= sizeof(scan_packet_t)) {
    parser_idx = 0;
    return RESULT_OK;
  }

  rx_buffer[parser_idx++] = c;

  switch (parser_idx) {
  case 1:
    if (c != START_BYTE0) {
      parser_idx = 0;
    }
    break;

  case 2:
    if (c != START_BYTE1) {
      parser_idx = 0;
    }
    break;

  case 3:
    if (c != START_BYTE2) {
      parser_idx = 0;
    }
    break;

  case 4:
    if (c != SAMPLES_PER_PACKET) {
      //parser_idx = 0;
      result = ERROR_INVALID_PACKET;
    }
    break;

  case 5: // speed LSB
  case 6: // speed MSB
    break;

  case 7: // start angle LSB
  case 8: // start angle MSB
    break;

  default:
    if (parser_idx > sizeof(scan_packet_t)) {
      result = ERROR_INVALID_PACKET;
    }
    break;

  case sizeof(scan_packet_t) - 3: // end angle LSB
  case sizeof(scan_packet_t) - 2: // end angle MSB
    break;

  case sizeof(scan_packet_t) - 1: // CRC16 LSB
    break;

  case sizeof(scan_packet_t) - 0: // CRC16 MSB
    //result = ERROR_CHECKSUM;
    rotation_speed = decodeUInt16(scan_packet.rotation_speed);
    uint16_t start_angle = decodeUInt16(scan_packet.start_angle);
    uint16_t end_angle = decodeUInt16(scan_packet.end_angle);

    if (start_angle < ANGLE_MIN || end_angle < ANGLE_MIN) {
      result = ERROR_INVALID_PACKET;
      break;
    }

    bool scan_completed_mid_packet = end_angle < start_angle;
    bool scan_completed_between_packets = start_angle < end_angle_prev;
    //bool scan_completed = start_angle < start_angle_prev;
    bool scan_completed = scan_completed_mid_packet || scan_completed_between_packets;
    //start_angle_prev = start_angle;
    end_angle_prev = end_angle;

    postPacket(rx_buffer, sizeof(scan_packet_t), scan_completed);

    static float constexpr ONE_OVER_64 = 1.0 / 64;
    float start_angle_deg = (start_angle - ANGLE_MIN) * ONE_OVER_64;
    float end_angle_deg = (end_angle - ANGLE_MIN) * ONE_OVER_64;
    if (start_angle > end_angle)
      end_angle_deg += 360;

    static float constexpr ONE_OVER_7 = 1.0 / (SAMPLES_PER_PACKET - 1);
    float step_deg = end_angle_deg - start_angle_deg;
    step_deg *= ONE_OVER_7;

    float angle_deg_prev = start_angle_deg;
    for (uint8_t i = 0; i < SAMPLES_PER_PACKET; i++) {
      float distance_mm = (int16_t) decodeUInt16(scan_packet.sample[i].distance_mm);
      float quality = scan_packet.sample[i].quality;

      float angle_deg = start_angle_deg + step_deg * i;

      scan_completed = false;
      if (scan_completed_mid_packet) {
        scan_completed = (angle_deg >= 360 && angle_deg_prev < 360);
      } else if (scan_completed_between_packets) {
        scan_completed = (i == 0);
      }
      angle_deg_prev = angle_deg;

      angle_deg = angle_deg > 360 ? angle_deg - 360 : angle_deg;
      //distance_mm = distance_mm < 0 ? 0 : distance_mm;

      postScanPoint(angle_deg, distance_mm, quality, scan_completed);
      //scan_completed = false;
    }
    parser_idx = 0;
    break;
  }

  if (result < RESULT_OK)
    parser_idx = 0;

  return result;
}

uint16_t LDS_CAMSENSE_X1::decodeUInt16(const uint16_t value) const {
  union {
    uint16_t i;
    char c[2];
  } bint = {0x0201};

  return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
}

const char* LDS_CAMSENSE_X1::getModelName() { return "Camsense X1"; }