// Copyright 2023-2026 KAIA.AI
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
// COIN-D4A driver, ported from QuirkyCort's MicroPython driver:
//   https://github.com/QuirkyCort/IoTy/blob/main/public/extensions/coind4.py
// See LDS_COIN_D4A.h for details and credit.

#include "LDS_COIN_D4A.h"

void LDS_COIN_D4A::init() {
  scanning = false;
  scan_freq_hz = 0;
  start_angle_x64_prev = 0;
  sample_count = 0;
  parser_idx = 0;
}

LDS::result_t LDS_COIN_D4A::start() {
  postInfo(INFO_MODEL, getModelName());
  sendCommand(CMD_START);
  scanning = true;
  return RESULT_OK;
}

LDS::result_t LDS_COIN_D4A::stop() {
  sendCommand(CMD_STOP);
  scanning = false;
  return RESULT_OK;
}

void LDS_COIN_D4A::sendCommand(uint8_t code) {
  uint8_t cmd[4] = { HEADER_0, HEADER_1, code,
                     (uint8_t)(HEADER_0 ^ HEADER_1 ^ code) };
  writeSerial(cmd, sizeof(cmd));
}

void LDS_COIN_D4A::setHighExposure() { sendCommand(CMD_HIGH_EXPOSURE); }
void LDS_COIN_D4A::setLowExposure() { sendCommand(CMD_LOW_EXPOSURE); }

uint32_t LDS_COIN_D4A::getSerialBaudRate() {
  return 115200; // CSPC M1C1-Mini family default; confirm on hardware
}

float LDS_COIN_D4A::getCurrentScanFreqHz() {
  return scan_freq_hz;
}

float LDS_COIN_D4A::getTargetScanFreqHz() {
  return DEFAULT_SCAN_FREQ;
}

int LDS_COIN_D4A::getSamplingRateHz() {
  return 2000; // guesstimate; ~10Hz spin, undocumented points-per-rev
}

bool LDS_COIN_D4A::isActive() {
  return scanning;
}

LDS::result_t LDS_COIN_D4A::setScanTargetFreqHz(float freq) {
  // The COIN-D4A spins at a fixed ~10Hz; the scan rate is not host-settable.
  return ERROR_NOT_IMPLEMENTED;
}

void LDS_COIN_D4A::loop() {
  while (true) {
    int c = readSerial();
    if (c < 0)
      break;

    result_t result = processByte((uint8_t)c);
    if (result < 0)
      postError(result, "");
  }
}

LDS::result_t LDS_COIN_D4A::processByte(uint8_t c) {
  // Sync to the 0xAA 0x55 header.
  if (parser_idx == 0) {
    if (c == HEADER_0)
      rx_buffer[parser_idx++] = c;
    return RESULT_OK;
  }

  if (parser_idx == 1) {
    if (c == HEADER_1) {
      rx_buffer[parser_idx++] = c;
    } else if (c != HEADER_0) {
      parser_idx = 0; // neither 0x55 nor another 0xAA: resync
    }
    // (c == HEADER_0) keeps parser_idx == 1, rx_buffer[0] already 0xAA
    return RESULT_OK;
  }

  rx_buffer[parser_idx++] = c;

  if (parser_idx == 4) {
    sample_count = rx_buffer[3];
    if (sample_count == 0 || sample_count > MAX_SAMPLES) {
      parser_idx = 0;
      return ERROR_INVALID_PACKET;
    }
  }

  // Full frame received?
  if (parser_idx >= HEADER_LEN &&
      parser_idx == (uint16_t)(HEADER_LEN + sample_count * 3)) {
    LDS::result_t result = RESULT_OK;

    // Two split XOR checksums: byte 8 over even-indexed bytes, byte 9 over odd.
    uint8_t cs_even = rx_buffer[0] ^ rx_buffer[2] ^ rx_buffer[4] ^ rx_buffer[6];
    uint8_t cs_odd  = rx_buffer[1] ^ rx_buffer[3] ^ rx_buffer[5] ^ rx_buffer[7];
    for (uint8_t i = 0; i < sample_count; i++) {
      uint8_t base = HEADER_LEN + i * 3;
      cs_even ^= rx_buffer[base] ^ rx_buffer[base + 1];
      cs_odd  ^= rx_buffer[base + 2];
    }

    if (cs_even != rx_buffer[8] || cs_odd != rx_buffer[9])
      result = ERROR_CHECKSUM;
    else
      processFrame();

    parser_idx = 0;
    sample_count = 0;
    return result;
  }

  if (parser_idx >= FRAME_LEN) { // safety net; should not happen
    parser_idx = 0;
    sample_count = 0;
  }
  return RESULT_OK;
}

void LDS_COIN_D4A::processFrame() {
  uint8_t speed_byte = rx_buffer[2];
  if (speed_byte & 0x01)
    scan_freq_hz = (speed_byte >> 1) * 0.1f; // rev/s == Hz

  int32_t start_angle = (int32_t)(((uint16_t)rx_buffer[5] << 8) | rx_buffer[4]) >> 1;
  int32_t end_angle   = (int32_t)(((uint16_t)rx_buffer[7] << 8) | rx_buffer[6]) >> 1;
  if (end_angle < start_angle)
    start_angle -= DEG_PER_REV_X64; // wrapped through 360 deg

  int32_t step = (sample_count > 1) ?
                 (end_angle - start_angle) / (sample_count - 1) : 0;

  // Detect a new rotation by a drop in the (normalized) start angle.
  uint16_t start_norm = (uint16_t)(((start_angle % DEG_PER_REV_X64)
                                    + DEG_PER_REV_X64) % DEG_PER_REV_X64);
  bool new_rotation = start_norm < start_angle_x64_prev;
  start_angle_x64_prev = start_norm;

  postPacket(rx_buffer, parser_idx, new_rotation);

  for (uint8_t i = 0; i < sample_count; i++) {
    uint8_t base = HEADER_LEN + i * 3;
    uint8_t b1 = rx_buffer[base + 1];
    uint8_t b2 = rx_buffer[base + 2];
    uint16_t distance_mm = (uint16_t)(b1 >> 2) | ((uint16_t)b2 << 6); // 14-bit
    float quality = (float)((b2 >> 2) | ((b1 & 0x03) << 6));

    int32_t angle_x64 = start_angle + (int32_t)i * step;
    float angle_deg = angle_x64 / 64.0f;
    while (angle_deg < 0)      angle_deg += 360.0f;
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;

    bool scan_completed = (i == 0) && new_rotation;
    postScanPoint(angle_deg, distance_mm, quality, scan_completed);
  }
}

const char* LDS_COIN_D4A::getModelName() { return "COIN-D4A"; }
