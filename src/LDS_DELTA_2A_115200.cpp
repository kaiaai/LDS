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

#include "LDS_DELTA_2A_115200.h"

void LDS_DELTA_2A_115200::init() {
  motor_enabled = false;
  pwm_val = 0.6;
  scan_freq_hz = 0;
  scan_freq_hz_setpoint = get_default_scan_freq_hz();
  parser_state = 0;
  checksum = 0;

  uint16_t max_data_samples = get_max_data_sample_count();
  uint16_t max_data_len_bytes = (uint16_t) (sizeof(meas_sample_t) * max_data_samples);
  rx_buffer = new uint8_t[max_data_len_bytes];

  scanFreqPID.init(&scan_freq_hz, &pwm_val, &scan_freq_hz_setpoint, 3.0e-1, 1.0e-1, 0.0, PID_v1::DIRECT);
  scanFreqPID.SetOutputLimits(0, 1.0);
  scanFreqPID.SetSampleTime(20);
  scanFreqPID.SetMode(PID_v1::AUTOMATIC);
  enableMotor(false);
}

uint16_t LDS_DELTA_2A_115200::get_max_data_sample_count() {
  return 28;
}

float LDS_DELTA_2A_115200::get_default_scan_freq_hz() {
  return 6; // 5.25f; // guesstimate
}

LDS::result_t LDS_DELTA_2A_115200::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return RESULT_OK;
}

uint32_t LDS_DELTA_2A_115200::getSerialBaudRate() {
  return 115200;
}

float LDS_DELTA_2A_115200::getTargetScanFreqHz() {
  return scan_freq_hz_setpoint;
}

int LDS_DELTA_2A_115200::getSamplingRateHz() {
  return 1890; // guesstimate
}

LDS::result_t LDS_DELTA_2A_115200::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  scanFreqPID.SetTunings(Kp, Ki, Kd);
  return RESULT_OK;
}

LDS::result_t LDS_DELTA_2A_115200::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  scanFreqPID.SetSampleTime(sample_period_ms);
  return RESULT_OK;
}

float LDS_DELTA_2A_115200::getCurrentScanFreqHz() {
  return scan_freq_hz;
}

LDS::result_t LDS_DELTA_2A_115200::stop() {
  enableMotor(false);
  return RESULT_OK;
}

void LDS_DELTA_2A_115200::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    setMotorPin(DIR_OUTPUT_PWM, LDS_MOTOR_PWM_PIN);
    setMotorPin(enable ? pwm_val : float(VALUE_LOW), LDS_MOTOR_PWM_PIN);
  } else {
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }
}

bool LDS_DELTA_2A_115200::isActive() {
  return motor_enabled;
}

LDS::result_t LDS_DELTA_2A_115200::setScanTargetFreqHz(float freq) {
  scan_freq_hz_setpoint = freq <= 0 ? get_default_scan_freq_hz() : freq;
  return RESULT_OK;
}

void LDS_DELTA_2A_115200::loop() {
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

LDS::result_t LDS_DELTA_2A_115200::processByte(uint8_t c) {
  uint16_t packet_length = 0;
  uint16_t data_length = 0;
  LDS::result_t result = RESULT_OK;
  packet_header_t * packet_header = (packet_header_t *)rx_buffer;

  uint16_t max_data_samples = get_max_data_sample_count();
  uint16_t max_data_len_bytes = (uint16_t) (sizeof(meas_sample_t) * max_data_samples);
  uint16_t max_packet_len_bytes_less_crc = max_data_len_bytes + sizeof(packet_header_t);

  if (parser_idx >= max_packet_len_bytes_less_crc) {
    parser_idx = 0;
    return RESULT_OK;//ERROR_INVALID_PACKET;
  }

  rx_buffer[parser_idx++] = c;
  checksum += c;

  switch (parser_idx) {
  case 1:
    if (c != START_BYTE) {
      //result = ERROR_INVALID_VALUE;
      parser_idx = 0;
    } else
      checksum = c;
    break;

  case 2:
    break;

  case 3:
    packet_length = decodeUInt16(packet_header->packet_length);
    if (packet_length > max_packet_len_bytes_less_crc + sizeof(uint16_t))
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
    data_length = decodeUInt16(packet_header->data_length);
    if (data_length == 0 || data_length > max_data_len_bytes)
      result = ERROR_INVALID_PACKET;
    break;

  default:
    // Keep reading
    packet_length = decodeUInt16(packet_header->packet_length);
    if (parser_idx != packet_length + 2)
      break;

    uint16_t pkt_checksum = (rx_buffer[parser_idx-2] << 8) + rx_buffer[parser_idx-1];

    pkt_checksum += rx_buffer[parser_idx-2];
    pkt_checksum += rx_buffer[parser_idx-1];
    if (checksum != pkt_checksum) {
      result = ERROR_CHECKSUM;
      break;
    }

    scan_freq_hz = packet_header->scan_freq_x20 * 0.05;
    uint8_t packets_per_scan = get_packets_per_scan();

    if (packet_header->data_type == DATA_TYPE_RPM_AND_MEAS) {
      uint16_t start_angle_x100 = decodeUInt16(packet_header->start_angle_x100);
      bool scan_completed = start_angle_x100 == 0;

      postPacket(rx_buffer, parser_idx, scan_completed);

      data_length = decodeUInt16(packet_header->data_length);
      if (data_length < 8) {
        result = ERROR_INVALID_PACKET;
        break;
      }

      uint16_t sample_count = (data_length - 5) / 3;
      if (sample_count > max_data_samples) {
        result = ERROR_INVALID_PACKET;
        break;
      }

      //int16_t offset_angle_x100 = (int16_t)decodeUInt16((uint16_t)packet_header->offset_angle_x100);
      float start_angle = start_angle_x100 * 0.01;
      //start_angle += offset_angle_x100 * 0.01;
      //DEG_PER_PACKET = 360.0f / (float)PACKETS_PER_SCAN; // 22.5 deg

      float coeff = 360.0f / (packets_per_scan * sample_count);
//      meas_sample_t * samples = (meas_sample_t *)&packet_header[sizeof(packet_header_t)];
      meas_sample_t * samples = (meas_sample_t *)(packet_header + sizeof(packet_header_t));
      for (uint16_t idx = 0; idx < sample_count; idx++) {
        float angle_deg = start_angle + idx * coeff;

        uint16_t distance_mm_x4 = decodeUInt16(samples[idx].distance_mm_x4);
        float distance_mm = distance_mm_x4 * 0.25;
        float quality = samples[idx].quality;
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

uint16_t LDS_DELTA_2A_115200::decodeUInt16(const uint16_t value) const {
  union {
    uint16_t i;
    char c[2];
  } bint = {0x0102};

  return bint.c[0] == 0x01 ? value : (value << 8) + (value >> 8);
}

const char* LDS_DELTA_2A_115200::getModelName() {
  return "3irobotics Delta-2A 115200 baud";
}

uint8_t LDS_DELTA_2A_115200::get_packets_per_scan(){
  return 16;
}
