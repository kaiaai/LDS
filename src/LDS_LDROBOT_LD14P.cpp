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

#include "LDS_LDROBOT_LD14P.h"

void LDS_LDROBOT_LD14P::init() {
  motor_enabled = false;
  speed_deg_per_sec = 0;
  parser_idx = 0;
  crc = 0;
  end_angle_deg_x100_prev = 0;
  target_scan_freq = DEFAULT_SCAN_FREQ;

  enableMotor(false);
}

LDS::result_t LDS_LDROBOT_LD14P::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return RESULT_OK;
}

uint32_t LDS_LDROBOT_LD14P::getSerialBaudRate() {
  return 230400;
}

float LDS_LDROBOT_LD14P::getTargetScanFreqHz() {
  return target_scan_freq;
}

int LDS_LDROBOT_LD14P::getSamplingRateHz() {
  return 4000;
}

float LDS_LDROBOT_LD14P::getCurrentScanFreqHz() {
  static constexpr float ONE_OVER_360 = 1.0f/360.0f;
  return ONE_OVER_360 * speed_deg_per_sec;
}

LDS::result_t LDS_LDROBOT_LD14P::stop() {
  enableMotor(false);
  return RESULT_OK;
}

void LDS_LDROBOT_LD14P::enableMotor(bool enable) {
  static constexpr uint8_t START_CMD[] = {0x54, 0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x5E};
  static constexpr uint8_t STOP_CMD[] = {0x54, 0xA1, 0x04, 0x00, 0x00, 0x00, 0x00, 0x4A};
  motor_enabled = enable;

  if (enable) {
    writeSerial(START_CMD, sizeof(START_CMD));
  } else {
    writeSerial(STOP_CMD, sizeof(STOP_CMD));
  }
  // TODO verify serial response
}

bool LDS_LDROBOT_LD14P::isActive() {
  return motor_enabled;
}

LDS::result_t LDS_LDROBOT_LD14P::setScanTargetFreqHz(float freq) {
  uint8_t SET_SCAN_SPEED_CMD[] = {0x54, 0xA2, 0x04, 0x70, 0x08, 0x00, 0x00, 0xA1};

  if (freq <= 0)
    freq = DEFAULT_SCAN_FREQ;

  if (freq < 2 || freq > 8)
    return ERROR_INVALID_VALUE;

  if (freq == target_scan_freq)
    return RESULT_OK;

  uint16_t deg_per_sec = uint16_t(round(freq * 360));
  SET_SCAN_SPEED_CMD[3] = uint8_t(deg_per_sec & 0xff);
  SET_SCAN_SPEED_CMD[4] = uint8_t((deg_per_sec >> 8) & 0xff);

  uint8_t cmd_crc = 0;
  for (uint8_t i = 0; i < sizeof(SET_SCAN_SPEED_CMD)-1; i++)
    cmd_crc = checkSum(SET_SCAN_SPEED_CMD[i], cmd_crc);
  SET_SCAN_SPEED_CMD[sizeof(SET_SCAN_SPEED_CMD)-1] = cmd_crc;

  writeSerial(SET_SCAN_SPEED_CMD, sizeof(SET_SCAN_SPEED_CMD));
  target_scan_freq = freq;

  return RESULT_OK;
}

void LDS_LDROBOT_LD14P::loop() {
  while (true) {
    int c = readSerial();
    if (c < 0)
      break;

    result_t result = processByte((uint8_t)c);
    if (result < 0)
      postError(result, "");
  }
}

inline uint8_t LDS_LDROBOT_LD14P::checkSum(uint8_t value, uint8_t crc) {
  static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
    0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
    0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
    0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
    0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
    0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
    0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
    0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
    0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
    0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
    0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
    0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
    0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
    0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
    0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
    0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
    0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
    0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
  };
  return CRC_TABLE[(crc ^ value) & 0xff];
}

LDS::result_t LDS_LDROBOT_LD14P::processByte(uint8_t c) {
  LDS::result_t result = RESULT_OK;
  uint8_t * rx_buffer = (uint8_t *)&scan_packet;

  if (parser_idx >= sizeof(scan_packet_t)) {
    parser_idx = 0;
    return RESULT_OK;
  }

  crc = parser_idx == 0 ? 0 : crc;
  rx_buffer[parser_idx++] = c;
  if (parser_idx < 47)
    crc = checkSum(c, crc);

  switch (parser_idx) {
  case 1:
    if (c != START_BYTE) {
      parser_idx = 0;
    }
    break;

  case 2:
    if (c != VER_LEN) {
      parser_idx = 0;
    }
    break;

  case 3: // speed LSB
  case 4: // speed MSB
    break;

  case 5: // start angle LSB
  case 6: // start angle MSB
    break;

  default:
    if (parser_idx > 6 && parser_idx <= (6 + DATA_BYTE_LEN)) {
    } else {
      result = ERROR_INVALID_PACKET;
    }
    break;

  case 43: // end angle LSB
  case 44: // end angle MSB
    break;

  case 45: // timestamp LSB
  case 46: // end angle MSB
    break;

  case 47: // CRC
    if (crc != scan_packet.crc8) {
      result = ERROR_CHECKSUM;
    } else {

      speed_deg_per_sec = decodeUInt16(scan_packet.speed_deg_per_sec);
      uint16_t start_angle_deg_x100 = decodeUInt16(scan_packet.start_angle_deg_x100);
      uint16_t end_angle_deg_x100 = decodeUInt16(scan_packet.end_angle_deg_x100);

      bool scan_completed_mid_packet = end_angle_deg_x100 < start_angle_deg_x100;
      bool scan_completed_between_packets = start_angle_deg_x100 < end_angle_deg_x100_prev;
      bool scan_completed = scan_completed_mid_packet || scan_completed_between_packets;
      end_angle_deg_x100_prev = end_angle_deg_x100;

      postPacket(rx_buffer, parser_idx, scan_completed);

      float start_angle = start_angle_deg_x100*0.01f;
      float end_angle = end_angle_deg_x100*0.01f;

      if (end_angle < start_angle)
        end_angle = end_angle + 360;

      static constexpr float _1_OVER_PPP_M1 = 1.0f / (POINTS_PER_PACK - 1);
      float step_angle = (end_angle - start_angle)*_1_OVER_PPP_M1;

      float angle_deg_prev = start_angle;
      float last_shift_delta = 0;
      for (uint8_t i = 0; i < POINTS_PER_PACK; i++) {
        float angle_deg = start_angle + step_angle*i;
        float distance_mm = decodeUInt16(scan_packet.sample[i].distance_mm);
        float quality = scan_packet.sample[i].intensity;

        // https://github.com/ldrobotSensorTeam/ldlidar_sl_sdk.git sl_transform.cpp
        float offset_x_ = 5.9f;
        float offset_y_ = -18.975571f;
        float angle_corrected;
        if (distance_mm > 0) {
          float x = distance_mm + offset_x_;
          float y = distance_mm * 0.11923f + offset_y_;
          float shift = atan(y / x) * RAD_TO_DEG;
          angle_corrected = angle_deg - shift;
          last_shift_delta = shift;
        } else {
          angle_corrected = angle_deg - last_shift_delta;
        }

        scan_completed = false;
        if (scan_completed_mid_packet) {
          scan_completed = (angle_deg >= 360 && angle_deg_prev < 360);
        } else if (scan_completed_between_packets) {
          scan_completed = (i == 0);
        }

        if (angle_corrected > 360)
          angle_corrected -= 360;
        if (angle_corrected < 0)
          angle_corrected += 360;

        postScanPoint(angle_corrected, distance_mm, quality, scan_completed);
        angle_deg_prev = angle_deg;
      }
    }
    parser_idx = 0;
    break;
  }

  if (result < RESULT_OK)
    parser_idx = 0;

  return result;
}

uint16_t LDS_LDROBOT_LD14P::decodeUInt16(const uint16_t value) const {
  union {
    uint16_t i;
    char c[2];
  } bint = {0x0201};

  return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
}

const char* LDS_LDROBOT_LD14P::getModelName() { return "LDROBOT LD14P"; }