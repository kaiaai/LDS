// Copyright 2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM
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

#pragma once
#include "LDS.h"

class LDS_LDROBOT_LD14P : public LDS {
  public:
    virtual void init() override;

    virtual result_t start() override;
    virtual result_t stop() override;
    virtual void loop() override; // Call loop() frequently from Arduino loop()

    virtual uint32_t getSerialBaudRate() override;
    virtual float getCurrentScanFreqHz() override;
    virtual float getTargetScanFreqHz() override;
    virtual int getSamplingRateHz() override;
    virtual bool isActive() override;
    virtual const char* getModelName() override;

    virtual result_t setScanTargetFreqHz(float freq) override;

  protected:
    static const uint8_t START_BYTE = 0x54;
    static const uint8_t POINTS_PER_PACK = 12;
    static const uint8_t VER_LEN = 0x2C;
    static constexpr float DEFAULT_SCAN_FREQ = 6.0f;

    struct meas_sample_t {
      uint16_t distance_mm;
      uint8_t intensity;
    } __attribute__((packed));

    static const uint16_t DATA_BYTE_LEN = sizeof(meas_sample_t) * POINTS_PER_PACK;

    struct scan_packet_t {
      uint8_t start_byte;
      uint8_t ver_len;
      uint16_t speed_deg_per_sec;
      uint16_t start_angle_deg_x100;
      meas_sample_t sample[POINTS_PER_PACK];
      uint16_t end_angle_deg_x100;
      uint16_t timestamp_ms;
      uint8_t crc8;
    } __attribute__((packed));

    virtual void enableMotor(bool enable);
    LDS::result_t processByte(uint8_t c);
    uint16_t decodeUInt16(const uint16_t value) const;
    uint8_t checkSum(uint8_t value, uint8_t crc);

    bool motor_enabled;
    uint16_t speed_deg_per_sec;
    scan_packet_t scan_packet;
    uint16_t parser_idx;
    uint8_t crc;
    uint16_t end_angle_deg_x100_prev;
    float target_scan_freq;
};
