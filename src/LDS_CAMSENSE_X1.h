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

class LDS_CAMSENSE_X1 : public LDS {
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
    static const uint8_t START_BYTE0 = 0x55;
    static const uint8_t START_BYTE1 = 0xAA;
    static const uint8_t START_BYTE2 = 0x03;
    static const uint8_t SAMPLES_PER_PACKET = 0x08;
    static const uint16_t ANGLE_MIN = 0xA000;

    struct meas_sample_t {
      int16_t distance_mm;
      uint8_t quality;
    } __attribute__((packed));

    struct scan_packet_t {
      uint8_t start_byte0; // 0x55
      uint8_t start_byte1; // 0xAA
      uint8_t start_byte2; // 0x03
      uint8_t samples_per_packet; // 0x08
      uint16_t rotation_speed; // Hz*64*60
      uint16_t start_angle;
      meas_sample_t sample[SAMPLES_PER_PACKET];
      uint16_t end_angle;
      uint16_t crc16;
    } __attribute__((packed));

    LDS::result_t processByte(uint8_t c);
    uint16_t decodeUInt16(const uint16_t value) const;

    uint16_t rotation_speed;
    scan_packet_t scan_packet;
    uint16_t parser_idx;
    //uint16_t start_angle_prev;
    uint16_t end_angle_prev;
};
