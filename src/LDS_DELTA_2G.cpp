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

#include "LDS_DELTA_2G.h"

void LDS_DELTA_2G::init() {
  motor_enabled = false;
  pwm_val = 0.6;
  scan_freq_hz = 0;
  scan_freq_hz_setpoint = DEFAULT_SCAN_FREQ_HZ;
  parser_state = 0;
  checksum = 0;

  scanFreqPID.init(&scan_freq_hz, &pwm_val, &scan_freq_hz_setpoint, 3.0e-1, 1.0e-1, 0.0, PID_v1::DIRECT);
  scanFreqPID.SetOutputLimits(0, 1.0);
  scanFreqPID.SetSampleTime(20);
  scanFreqPID.SetMode(PID_v1::AUTOMATIC);
  enableMotor(false);
}

LDS::result_t LDS_DELTA_2G::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return RESULT_OK;
}

uint32_t LDS_DELTA_2G::getSerialBaudRate() {
  return 115200;
}

float LDS_DELTA_2G::getTargetScanFreqHz() {
  return scan_freq_hz_setpoint;
}

int LDS_DELTA_2G::getSamplingRateHz() {
  return 1890;
}

LDS::result_t LDS_DELTA_2G::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  scanFreqPID.SetTunings(Kp, Ki, Kd);
  return RESULT_OK;
}

LDS::result_t LDS_DELTA_2G::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  scanFreqPID.SetSampleTime(sample_period_ms);
  return RESULT_OK;
}

float LDS_DELTA_2G::getCurrentScanFreqHz() {
  return scan_freq_hz;
}

void LDS_DELTA_2G::stop() {
  enableMotor(false);
}

void LDS_DELTA_2G::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    setMotorPin(DIR_OUTPUT_PWM, LDS_MOTOR_PWM_PIN);
    setMotorPin(enable ? pwm_val : float(VALUE_LOW), LDS_MOTOR_PWM_PIN);
  } else {
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }
}

bool LDS_DELTA_2G::isActive() {
  return motor_enabled;
}

LDS::result_t LDS_DELTA_2G::setScanTargetFreqHz(float freq) {
  if (freq <= 0) {
    scan_freq_hz_setpoint = DEFAULT_SCAN_FREQ_HZ;
    return RESULT_OK;
  }
  
  if (freq <= DEFAULT_SCAN_FREQ_HZ*0.9f || freq >= DEFAULT_SCAN_FREQ_HZ*1.1f)
    return ERROR_INVALID_VALUE;

  scan_freq_hz_setpoint = freq;
  return RESULT_OK;
}

void LDS_DELTA_2G::loop() {
  while (true) {
    int c = readSerial();
    if (c < 0)
      break;

    result_t result = processByte((uint8_t)c);
    if (result < 0)
      postError(result, "");
  }

  if (motor_enabled) {
    scanFreqPID.Compute();

    if (pwm_val != pwm_last) {
      setMotorPin(pwm_val, LDS_MOTOR_PWM_PIN);
      pwm_last = pwm_val;
    }
  }
}

LDS::result_t LDS_DELTA_2G::processByte(uint8_t c) {
  uint16_t packet_length = 0;
  uint16_t data_length = 0;
  LDS::result_t result = RESULT_OK;
  uint8_t * rx_buffer = (uint8_t *)&scan_packet;

  if (parser_idx >= sizeof(scan_packet_t)) {
    parser_idx = 0;
    return ERROR_INVALID_PACKET;
  }

  rx_buffer[parser_idx++] = c;
  checksum += c;

  switch (parser_idx) {
  case 1:
    if (c != START_BYTE)
      result = ERROR_INVALID_VALUE;
    else
      checksum = c;
    break;

  case 2:
    break;

  case 3:
    packet_length = decodeUInt16(scan_packet.packet_length);
    if (packet_length > sizeof(scan_packet_t) - sizeof(scan_packet.checksum))
      result = ERROR_INVALID_PACKET;
    break;

  case 4:
    if (c != PROTOCOL_VERSION)
      result = ERROR_INVALID_VALUE;
    break;

  case 5:
    if (c != PACKET_TYPE)
      result = ERROR_INVALID_VALUE;
    break;

  case 6:
    if (c != DATA_TYPE_RPM_AND_MEAS && c != DATA_TYPE_RPM_ONLY)
      result = ERROR_INVALID_VALUE;
    break;

  case 7: // data length MSB
    break;

  case 8: // data length LSB
    data_length = decodeUInt16(scan_packet.data_length);
    if (data_length == 0 || data_length > MAX_DATA_BYTE_LEN)
	    result = ERROR_INVALID_PACKET;
    break;

  default:
    // Keep reading
    packet_length = decodeUInt16(scan_packet.packet_length);
    if (parser_idx != packet_length + 2)
      break;

    uint16_t pkt_checksum = (rx_buffer[parser_idx-2] << 8) + rx_buffer[parser_idx-1];

    pkt_checksum += rx_buffer[parser_idx-2];
    pkt_checksum += rx_buffer[parser_idx-1];
    if (checksum != pkt_checksum) {
      result = ERROR_CHECKSUM;
      break;
    }

    scan_freq_hz = scan_packet.scan_freq_x20 * 0.05;

    if (scan_packet.data_type == DATA_TYPE_RPM_AND_MEAS) {
      uint16_t start_angle_x100 = decodeUInt16(scan_packet.start_angle_x100);
      bool scan_completed = start_angle_x100 == 0;

      postPacket(rx_buffer, parser_idx, scan_completed);

      data_length = decodeUInt16(scan_packet.data_length);
      if (data_length < 8) {
        result = ERROR_INVALID_PACKET;
        break;
      }

      uint16_t sample_count = (data_length - 5) / 3;
      if (sample_count > MAX_DATA_SAMPLES) {
        result = ERROR_INVALID_PACKET;
        break;
      }
      float start_angle = start_angle_x100 * 0.01;
      float coeff = DEG_PER_PACKET / (float)sample_count;
      for (uint16_t idx = 0; idx < sample_count; idx++) {
        float angle_deg = start_angle + idx * coeff;

        uint16_t distance_mm_x4 = decodeUInt16(scan_packet.sample[idx].distance_mm_x4);
        float distance_mm = distance_mm_x4 * 0.25;
        float quality = scan_packet.sample[idx].quality;
        postScanPoint(angle_deg, distance_mm, quality, scan_completed);
        scan_completed = false;
      }
    }
    parser_idx = 0;
    break;
  }

  if (result < RESULT_OK)
    parser_idx = 0;

  return result;
}

uint16_t LDS_DELTA_2G::decodeUInt16(const uint16_t value) const {
  union {
      uint16_t i;
      char c[2];
  } bint = {0x0102};

  return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
}

const char* LDS_DELTA_2G::getModelName() { return "3irobotics Delta-2G"; }