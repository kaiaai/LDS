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
// COIN-D4A (a.k.a. COIN-D4) driver. This is a small, cheap, self-spinning ToF
// LiDAR whose wire protocol matches the CSPC "M1C1 Mini" family
// (https://www.cspctech.com/resources).
//
// Ported from the MicroPython driver by QuirkyCort (Cort Stratton):
//   https://github.com/QuirkyCort/IoTy/blob/main/public/extensions/coind4.py
// Requested in https://github.com/kaiaai/awesome-2d-lidars/issues/3
//
// NOTE: not yet verified on physical hardware. The baud rate below is the CSPC
// M1C1-Mini family default; confirm it against your unit.

#pragma once
#include "LDS.h"

class LDS_COIN_D4A : public LDS {
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

    // COIN-D4A-specific: toggle the sensor's exposure setting
    void setHighExposure();
    void setLowExposure();

  protected:
    static const uint8_t HEADER_0 = 0xAA;
    static const uint8_t HEADER_1 = 0x55;
    static const uint8_t HEADER_LEN = 10;
    static const uint8_t MAX_SAMPLES = 25;
    static const uint8_t FRAME_LEN = HEADER_LEN + MAX_SAMPLES * 3; // 85
    static constexpr float DEFAULT_SCAN_FREQ = 10.0f;
    static const uint16_t DEG_PER_REV_X64 = 360 * 64; // 23040

    static const uint8_t CMD_START = 0xF0;
    static const uint8_t CMD_HIGH_EXPOSURE = 0xF1;
    static const uint8_t CMD_LOW_EXPOSURE = 0xF2;
    static const uint8_t CMD_STOP = 0xF5;

    void sendCommand(uint8_t code);
    LDS::result_t processByte(uint8_t c);
    void processFrame();

    bool scanning;
    float scan_freq_hz;
    uint16_t start_angle_x64_prev;
    uint8_t sample_count;
    uint16_t parser_idx;
    uint8_t rx_buffer[FRAME_LEN];
};
