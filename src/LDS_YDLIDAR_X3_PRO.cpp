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

#include "LDS_YDLIDAR_X3_PRO.h"

const char* LDS_YDLIDAR_X3_PRO::getModelName() { return "YDLIDAR X3 PRO"; }

LDS::result_t LDS_YDLIDAR_X3_PRO::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, getModelName());
  return LDS::RESULT_OK;
}

float LDS_YDLIDAR_X3_PRO::getCurrentScanFreqHz() {
  return motor_enabled ? float(scan_freq)/10 : 0;
}

uint32_t LDS_YDLIDAR_X3_PRO::getSerialBaudRate() {
  return 115200;
}

float LDS_YDLIDAR_X3_PRO::getTargetScanFreqHz() {
  return 7;
}

int LDS_YDLIDAR_X3_PRO::getSamplingRateHz() {
  return 4000;
}

void LDS_YDLIDAR_X3_PRO::stop() {
  enableMotor(false);
}

void LDS_YDLIDAR_X3_PRO::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    setMotorPin(DIR_INPUT, LDS_MOTOR_PWM_PIN);
  } else {
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }

  // Entire LDS on/off
  //setMotorPin(enable ? VALUE_HIGH : VALUE_LOW, LDS_MOTOR_PWM_PIN);
}