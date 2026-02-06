// Copyright 2023-2025 KAIA.AI
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
// EXPERIMENTAL - needs testing
// YDLIDAR T-mini Plus / T-mini Pro (ToF LiDAR)

#pragma once
#include "LDS_YDLIDAR_X4.h"

class LDS_YDLIDAR_TMINI : public LDS_YDLIDAR_X4 {
  public:
    virtual uint32_t getSerialBaudRate() override;
    virtual int getSamplingRateHz() override;
    virtual const char* getModelName() override;

  protected:
    virtual LDS::result_t waitScanDot() override;
};
