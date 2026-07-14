// Copyright 2023-2026 KAIA.AI
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
// LDS08RR, Camsense-protocol hardware revision.
//
// There is more than one LDS08RR revision in the wild. LDS_LDS08RR speaks the
// 3irobotix Delta protocol (0xAA framing); this one speaks the Camsense X1
// protocol family (0x55 0xAA framing) and will produce noise if parsed by the
// Delta driver, and vice versa. Pick the one that matches your unit: if your
// stream starts with 0x55 0xAA, use this class.
//
// Differences from LDS_CAMSENSE_X1:
//   - start_byte2        0x07 (X1: 0x03)
//   - samples_per_packet 12   (X1: 8)
//   - sample is 4 bytes: int16 distance + uint8 quality + 1 pad (X1: 3 bytes)
//   - packet is 60 bytes (X1: 36)
// Everything else matches: ANGLE_MIN 0xA000, angle = (raw - 0xA000)/64 deg,
// scan_freq_hz = rotation_speed/3840, distance is a signed int16 (negative =
// no return; 0x8000 observed as the "no return" marker), and the trailing
// 15-bit checksum (see checkSum(); LDS_CAMSENSE_X1 carries it but, unlike this
// driver, does not verify it).
//
// Reverse engineered from the capture in https://github.com/kaiaai/LDS/issues/17
// (thanks to Nelson / @npireso for the log). Verified against that 319KB,
// 3870-packet capture but NOT yet run against live hardware by us.

#pragma once
#include "LDS.h"

class LDS_LDS08RR_CAMSENSE : public LDS {
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
    static const uint8_t START_BYTE2 = 0x07;
    static const uint8_t SAMPLES_PER_PACKET = 0x0C; // 12
    static const uint16_t ANGLE_MIN = 0xA000;

    struct meas_sample_t {
      int16_t distance_mm; // signed; negative (0x8000) means no return
      uint8_t quality;
      uint8_t reserved;    // always 0 in the captures seen so far
    } __attribute__((packed));

    struct scan_packet_t {
      uint8_t start_byte0; // 0x55
      uint8_t start_byte1; // 0xAA
      uint8_t start_byte2; // 0x07
      uint8_t samples_per_packet; // 0x0C
      uint16_t rotation_speed; // Hz*64*60
      uint16_t start_angle;
      meas_sample_t sample[SAMPLES_PER_PACKET];
      uint16_t end_angle;
      uint16_t checksum; // 15-bit, see checkSum()
    } __attribute__((packed)); // 8 + 4*12 + 4 = 60

    LDS::result_t processByte(uint8_t c);
    uint16_t decodeUInt16(const uint16_t value) const;

    // 15-bit checksum over the packet's leading little-endian uint16 words,
    // excluding the trailing checksum field itself. This is 3irobotix/Camsense's
    // own algorithm, taken from HCLidar::checkDataCal() in the official Camsense
    // SDK (github.com/camsense/T2SDK, src/base/hclidar.cpp); it is the same
    // Neato-XV11-style fold used across this vendor family. Verified against
    // every packet of the issue #17 capture.
    uint16_t checkSum(const uint8_t * buffer, uint16_t length_bytes) const;

    uint16_t rotation_speed;
    scan_packet_t scan_packet;
    uint16_t parser_idx;
    uint16_t end_angle_prev;
};
