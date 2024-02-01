// Based on
//   Copyright 2014-2021 James LeRoy getSurreal.com
//   https://github.com/getSurreal/XV_Lidar_Controller
//
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

#include "LDS_LDS02RR.h"

void LDS_LDS02RR::init() {
  LDS_NEATO_XV11::init();
  cw = true;
}


LDS::result_t LDS_LDS02RR::start() {
  enableMotor(true);
  postInfo(INFO_MODEL, "LDS02RR");
  postInfo(INFO_SAMPLING_RATE, String(getSamplingRateHz()));
  postInfo(INFO_DEFAULT_TARGET_SCAN_FREQ_HZ, String(DEFAULT_SCAN_RPM/60.0f));
  return LDS::RESULT_OK;
}
