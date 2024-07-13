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

#include "LDS.h"

LDS::LDS() {
  scan_point_callback = NULL;
  packet_callback = NULL;
  serial_write_callback = NULL;
  serial_read_callback = NULL;
  motor_pin_callback = NULL;
  info_callback = NULL;
  error_callback = NULL;
}

void LDS::setScanPointCallback(ScanPointCallback scan_point_callback) {
  this->scan_point_callback = scan_point_callback; 
}

void LDS::setMotorPinCallback(MotorPinCallback motor_pin_callback) {
  this->motor_pin_callback = motor_pin_callback;
}

void LDS::setPacketCallback(PacketCallback packet_callback) {
  this->packet_callback = packet_callback; 
}

void LDS::setSerialReadCallback(SerialReadCallback serial_read_callback) {
  this->serial_read_callback = serial_read_callback; 
}

void LDS::setSerialWriteCallback(SerialWriteCallback serial_write_callback) {
  this->serial_write_callback = serial_write_callback; 
}

void LDS::setInfoCallback(InfoCallback info_callback) {
  this->info_callback = info_callback; 
}

void LDS::setErrorCallback(ErrorCallback error_callback) {
  this->error_callback = error_callback; 
}

void LDS::postScanPoint(float angle_deg, float dist_mm, float quality,
  bool scan_completed) {
  // dist_mm <=0 indicates invalid point
  if (scan_point_callback)
    scan_point_callback(angle_deg, dist_mm, quality, scan_completed);
}

void LDS::postPacket(uint8_t* data, uint16_t length, bool scan_completed) {
  if (packet_callback)
    packet_callback(data, length, scan_completed);
}

void LDS::setMotorPin(float value, lds_pin_t pin) {
  if (motor_pin_callback)
    motor_pin_callback(value, pin);
}

size_t LDS::writeSerial(const uint8_t * buffer, size_t length) {
  return (serial_write_callback) ? serial_write_callback(buffer, length) : 0;
}

int LDS::readSerial() {
  return (serial_read_callback) ? serial_read_callback() : ERROR_NOT_CONFIGURED;
}

void LDS::postInfo(info_t code, String info) {
  if (info_callback)
    info_callback(code, info);
}

void LDS::postError(result_t code, String aux_info) {
  if (error_callback)
    error_callback(code, aux_info);
}

LDS::result_t LDS::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  return ERROR_NOT_IMPLEMENTED;
}

LDS::result_t LDS::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  return ERROR_NOT_IMPLEMENTED;
}

String LDS::infoCodeToString(info_t code) {
  switch (code) {
    case INFO_MODEL:
      return "Model";
    case INFO_FIRMWARE_VERSION:
      return "Firmware Version";
    case INFO_HARDWARE_VERSION:
      return "Hardware Version";
    case INFO_SERIAL_NUMBER:
      return "Serial Number";
    case INFO_DEVICE_HEALTH:
      return "Device Health";
    case INFO_SAMPLING_RATE:
      return "Sampling Rate Hz";
    case INFO_DEFAULT_TARGET_SCAN_FREQ_HZ:
      return "Default Target Scan Frequency Hz";
    case INFO_OTHER:
      return "Other";
    default:
      return "Unknown";
  }
}

String LDS::resultCodeToString(result_t code) {
  switch (code) {
    case LDS::RESULT_OK:
      return "OK";
    case ERROR_TIMEOUT:
      return "Timeout error";
    case ERROR_INVALID_PACKET:
      return "Invalid Packet error";
    case ERROR_CHECKSUM:
      return "CRC error";
    case ERROR_NOT_READY:
      return "Not ready error";
    case ERROR_NOT_IMPLEMENTED:
      return "Not implemented error";
    case ERROR_NOT_CONFIGURED:
      return "Not configured error";
    case ERROR_MOTOR_DISABLED:
      return "Motor disabled error";
    case ERROR_INVALID_MODEL:
      return "Invalid model error";
    case ERROR_DEVICE_INFO:
      return "Device Info error";
    case ERROR_DEVICE_HEALTH:
      return "Device Health error";
    case ERROR_START_SCAN:
      return "Start Scan error";
    case ERROR_INVALID_VALUE:
      return "Invalid Value error";
    default:
      return "Unknown result code";
  }
}

String LDS::pinIDToString(lds_pin_t pin) {
  switch (pin) {
    case LDS_MOTOR_EN_PIN:
      return "MOTOR_EN_PIN";
    case LDS_MOTOR_PWM_PIN:
      return "MOTOR_PWM_PIN";
    default:
      return "Unknown pin";
  }
}

String LDS::pinStateToString(lds_pin_state_t state) {
  switch (state) {
    case VALUE_PWM:
      return "PWM";
    case VALUE_LOW:
      return "LOW";
    case VALUE_HIGH:
      return "HIGH";
    case DIR_INPUT:
      return "INPUT";
    case DIR_OUTPUT_CONST:
      return "OUTPUT CONST";
    case DIR_OUTPUT_PWM:
      return "OUTPUT PWM";
    default:
      return "Unknown state";
  }
}
