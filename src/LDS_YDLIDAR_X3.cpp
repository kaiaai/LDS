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

#include "LDS_YDLIDAR_X3.h"

const char* LDS_YDLIDAR_X3::getModelName() { return "YDLIDAR X3"; }

float LDS_YDLIDAR_X3::getCurrentScanFreqHz() {
  return motor_enabled ? scan_freq_hz : 0;
}

void LDS_YDLIDAR_X3::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    setMotorPin(DIR_INPUT, LDS_MOTOR_PWM_PIN);
  } else {
    // Cannot stop X3 completely
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }
}