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

#include "LDS_YDLIDAR_X2_X2L.h"

const char* LDS_YDLIDAR_X2_X2L::getModelName() { return "YDLIDAR X2/X2L"; }

int LDS_YDLIDAR_X2_X2L::getSamplingRateHz() {
  return 3000;
}