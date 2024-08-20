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
// Based on https://github.com/YDLIDAR/lidarCar/

#include "LDS_YDLIDAR_SCL.h"

const char* LDS_YDLIDAR_SCL::getModelName() { return "YDLIDAR SCL"; }

void LDS_YDLIDAR_SCL::init() {
  LDS_YDLIDAR_X4::init();

  pwm_val = PWM_START;
  scan_freq_setpoint_hz = DEFAULT_SCAN_FREQ_HZ;

  scanFreqPID.init(&scan_freq_hz, &pwm_val, &scan_freq_setpoint_hz, 3.0e-3, 1.0e-3, 0.0, PID_v1::DIRECT);
  scanFreqPID.SetOutputLimits(0, 1.0);
  scanFreqPID.SetSampleTime(20);
  scanFreqPID.SetMode(PID_v1::AUTOMATIC);
}

LDS::result_t LDS_YDLIDAR_SCL::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return LDS::RESULT_OK;
}

uint32_t LDS_YDLIDAR_SCL::getSerialBaudRate() {
  return 115200;
}

int LDS_YDLIDAR_SCL::getSamplingRateHz() {
  return 3000;
}

float LDS_YDLIDAR_SCL::getCurrentScanFreqHz() {
  return motor_enabled ? scan_freq_hz : 0;
}

LDS::result_t LDS_YDLIDAR_SCL::stop() {
  enableMotor(false);
  return RESULT_OK;
}

void LDS_YDLIDAR_SCL::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    //setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    //setMotorPin(VALUE_HIGH, LDS_MOTOR_PWM_PIN);
    setMotorPin(DIR_OUTPUT_PWM, LDS_MOTOR_PWM_PIN);
    setMotorPin(pwm_val, LDS_MOTOR_PWM_PIN);
  } else {
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }
}

void LDS_YDLIDAR_SCL::loop() {
  LDS_YDLIDAR_X4::loop();

  if (motor_enabled) {
    scanFreqPID.Compute();

    if (pwm_val != pwm_last) {
      setMotorPin(pwm_val, LDS_MOTOR_PWM_PIN);
      pwm_last = pwm_val;
    }
  }
}

LDS::result_t LDS_YDLIDAR_SCL::setScanTargetFreqHz(float freq) {
  scan_freq_setpoint_hz = freq <= 0 ? DEFAULT_SCAN_FREQ_HZ : freq;
  return RESULT_OK;
}

float LDS_YDLIDAR_SCL::getTargetScanFreqHz() {
  return scan_freq_setpoint_hz;
}

LDS::result_t LDS_YDLIDAR_SCL::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  scanFreqPID.SetTunings(Kp, Ki, Kd);
  return RESULT_OK;
}

LDS::result_t LDS_YDLIDAR_SCL::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  scanFreqPID.SetSampleTime(sample_period_ms);
  return RESULT_OK;
}