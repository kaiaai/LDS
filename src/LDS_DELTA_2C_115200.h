// Copyright 2026 sylvan-he
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

// EXPERIMENTAL: DELTA-2C PRO support for kaiaai/LDS.
// ------------------------------------------------
// This is a work-in-progress contribution to the kaiaai/LDS Arduino library.
// The protocol has been reverse-engineered from real captures, but the
// measurement precision has NOT been fully characterized. This code is provided
// in a "usable" state — it parses scans correctly, but treat all numbers as
// approximate pending calibration.
//
// Differences vs the 2A/2B/2D/2G variants (see PROTOCOL.md in the companion
// repository for full details):
//   - protocol version is 0x13 (not 0x01)
//   - the packet header carries an extra 2-byte end_angle after start_angle
//   - sample angle is interpolated between the explicit start/end angles
//
// Distance scale is 0.25 mm/unit, same as the Delta-2A family. The original
// contribution used 0.5 mm/unit (empirical); a second Delta-2C PRO unit's
// working decoder (rolandslepcik98, kaiaai/LDS#16) reports quarter-mm-quantized
// distances, showing 0.25 mm/unit is correct and 0.5 doubled every range.
//
// Motor note: the DELTA-2C PRO module has only M+/M- (motor power) and no
// PWM/EN input pin. The PID output below is intended for an external motor
// driver that modulates M+/M-. Alternatively, power M+/M- at a fixed voltage
// (~2.6 V gives ~7 Hz, where measurement frames appear; individual units
// vary) and leave the PWM pin unwired.

#pragma once
#include "LDS.h"
#include "PID_v1_0_0.h"

class LDS_DELTA_2C_115200 : public LDS {
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
    virtual result_t setScanPIDCoeffs(float Kp, float Ki, float Kd) override;
    virtual result_t setScanPIDSamplePeriodMs(uint32_t sample_period_ms) override;

  protected:
    static const uint8_t START_BYTE = 0xAA;
    // Delta-2C PRO uses protocol version 0x13 or 0x11, both with the identical
    // frame layout (0x01 is the different-layout 2A/2B/2D/2G family, rejected).
    // Two units confirmed: 0x13 (hapynfdj) and 0x11 (rolandslepcik98, poteytoman
    // — kaiaai/LDS#16, makerspet/support#95), verified byte-exact against both.
    static const uint8_t PROTOCOL_VERSION = 0x13;
    static const uint8_t PROTOCOL_VERSION_ALT = 0x11;
    static const uint8_t PACKET_TYPE = 0x61;
    static const uint8_t DATA_TYPE_RPM_AND_MEAS = 0xAD;
    static const uint8_t DATA_TYPE_RPM_ONLY = 0xAE;

    struct meas_sample_t {
      uint8_t quality;
      uint16_t distance_mm_x4; // 2C: 0.25 mm per unit (Delta-2A family scale)
    } __attribute__((packed));

    struct packet_header_t {
      uint8_t    start_byte; // 0xAA
      uint16_t   packet_length; // excludes checksum
      uint8_t    protocol_version; // 0x13
      uint8_t    packet_type; // 0x61
      uint8_t    data_type; // 0xAE -> data_length -> scan_freq_x20 -> no data
                            // 0xAD -> data_length -> scan_freq_x20 -> data
      uint16_t   data_length; // n_samples = (data_length - 7)/3;
      uint8_t    scan_freq_x20;
      int16_t    offset_angle_x100; // signed
      uint16_t   start_angle_x100; // unsigned
      uint16_t   end_angle_x100;   // 2C: extra field, absent in 2A/2B/2D/2G

      // meas_sample_t sample[MAX_DATA_SAMPLES]; // 3*28=84
      // uint16_t   checksum;
    } __attribute__((packed)); // 8 + 7 + 84 + 2 = 101

    virtual void enableMotor(bool enable);
    virtual LDS::result_t processByte(uint8_t c);
    virtual uint16_t get_max_data_sample_count();
    virtual float get_default_scan_freq_hz();
    virtual uint8_t get_packets_per_scan();

    uint16_t decodeUInt16(const uint16_t value) const;

    float scan_freq_hz_setpoint;
    bool motor_enabled;
    uint8_t parser_state;
    float pwm_val;
    float pwm_last;
    float scan_freq_hz;
    PID_v1 scanFreqPID;

    uint16_t parser_idx;
    uint16_t checksum;
    uint16_t last_start_angle_x100; // 2C: wrap detection for scan_completed
    uint8_t * rx_buffer;
};
