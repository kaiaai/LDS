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
// Based on
//   https://github.com/YDLIDAR/lidarCar/

#include "LDS_YDLIDAR_X2.h"

void LDS_YDLIDAR_X2::init() {
  ring_start_ms[0] = ring_start_ms[1] = 0;
  enableMotor(false);
}

LDS::result_t LDS_YDLIDAR_X2::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, "YDLIDAR X2");
  postInfo(INFO_SAMPLING_RATE, String(getSamplingRateHz()));
  postInfo(INFO_DEFAULT_TARGET_SCAN_FREQ_HZ, String(getTargetScanFreqHz()));
  return LDS::RESULT_OK;
}

uint32_t LDS_YDLIDAR_X2::getSerialBaudRate() {
  return 115200;
}

float LDS_YDLIDAR_X2::getTargetScanFreqHz() {
  return 7;
}

int LDS_YDLIDAR_X2::getSamplingRateHz() {
  return 3000;
}

void LDS_YDLIDAR_X2::stop() {
  enableMotor(false);
}