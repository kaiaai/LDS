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
// Based on https://github.com/getSurreal/XV_Lidar_Controller

#pragma once
#include "LDS.h"
#include "PID_v1_0_0.h"

class LDS_NEATO_XV11 : public LDS {
  public:
    void init() override;
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
    static constexpr float DEFAULT_SCAN_RPM = 300.0f;

    // REF: https://github.com/Xevel/NXV11/wiki
    // The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
    // It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
    // only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
    // data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
    // interrupted by the supports of the cover) .
    static const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"
    // The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
    // This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
    // size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
    // reflections (glass... ).
    static const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
    static const byte CRC_ERROR_FLAG = (1 << 0);

  protected:
    void enableMotor(bool enable);
    LDS::result_t processByte(int inByte);
    uint16_t processIndex();
    void processSpeed();
    uint8_t processDistance(int iQuad);
    void processSignalStrength(int iQuad);
    bool isValidPacket();
    void clearVars();

    float scan_rpm_setpoint; // desired scan RPM
    bool motor_enabled;
    int eState;

    // 90 packets * 22 bytes = 1,980 bytes per scan at 5Hz
    static const unsigned char COMMAND = 0xFA;        // Start of new packet
    static const int INDEX_LO = 0xA0;                 // lowest index value
    static const int INDEX_HI = 0xF9;                 // highest index value
    
    static const uint16_t N_DATA_QUADS = 4;                // there are 4 groups of data elements
    static const uint16_t N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB
    
    // Offsets to bytes within 'Packet'
    static const uint16_t OFFSET_TO_START = 0;
    static const uint16_t OFFSET_TO_INDEX = OFFSET_TO_START + 1;
    static const uint16_t OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
    static const uint16_t OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
    static const uint16_t OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
    static const uint16_t OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
    static const uint16_t OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
    static const uint16_t PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // 22 length of a complete packet
    // Offsets to the (4) elements of each of the (4) data quads
    static const uint16_t OFFSET_DATA_DISTANCE_LSB = 0;
    static const uint16_t OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
    static const uint16_t OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
    static const uint16_t OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;
    
    byte Packet[PACKET_LENGTH];                 // an input packet
    uint16_t ixPacket;                          // index into 'Packet' array

    static const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);
    
    static const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
    static const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
    
    float pwm_val;
    float pwm_last;
    float scan_rpm;
    PID_v1 scanFreqPID;
    
    uint16_t aryDist[N_DATA_QUADS];    // there are (4) distances, one for each data quad
                                       // so the maximum distance is 16383 mm (0x3FFF)
    uint16_t aryQuality[N_DATA_QUADS]; // same with 'quality'
    bool cw;
};
