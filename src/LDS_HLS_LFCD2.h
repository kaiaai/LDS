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
// Hitachi-LG HLS-LFCD2 / LDS-01 (TurtleBot3 LiDAR)
// Based on https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver

#pragma once
#include "LDS.h"

class LDS_HLS_LFCD2 : public LDS {
  public:
    virtual void init() override;

    virtual result_t start() override;
    virtual result_t stop() override;
    virtual void loop() override;

    virtual uint32_t getSerialBaudRate() override;
    virtual float getCurrentScanFreqHz() override;
    virtual float getTargetScanFreqHz() override;
    virtual int getSamplingRateHz() override;
    virtual bool isActive() override;
    virtual const char* getModelName() override;

    virtual result_t setScanTargetFreqHz(float freq) override;

  protected:
    static const uint8_t START_BYTE = 0xFA;
    static const uint8_t PACKETS_PER_SCAN = 60;
    static const uint8_t READINGS_PER_PACKET = 6;
    static const uint8_t PACKET_SIZE = 42;

    struct reading_t {
      uint16_t intensity;
      uint16_t range_mm;
    } __attribute__((packed));

    struct packet_t {
      uint8_t start_byte;      // 0xFA
      uint8_t index;           // 0xA0 + packet_number (0-59)
      uint16_t rpm;            // Motor speed
      reading_t readings[READINGS_PER_PACKET];  // 6 readings
      uint16_t checksum;
    } __attribute__((packed));

    bool motor_enabled;
    uint16_t motor_rpm;
    uint16_t parser_idx;
    uint8_t packet_index;
    packet_t packet;

    virtual void enableMotor(bool enable);
    LDS::result_t processByte(uint8_t c);
    uint16_t calcChecksum(uint8_t* data, uint16_t len);
};
