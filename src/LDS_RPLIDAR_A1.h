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
// Based on https://github.com/robopeak/rplidar_arduino

#pragma once
#include "LDS.h"

class LDS_RPLIDAR_A1 : public LDS {
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
    static uint16_t const DEFAULT_TIMEOUT_MS = 500;

    uint8_t const CMD_SCAN = 0x20;
    uint8_t const CMD_FORCE_SCAN = 0x21;
    uint8_t const CMD_STOP = 0x25;
    uint8_t const CMD_GET_DEV_INFO = 0x50;
    uint8_t const CMD_GET_DEV_HEALTH = 0x52;
    uint8_t const CMD_SYNC_BYTE = 0xA5;

    uint8_t const CMDFLAG_HAS_PAYLOAD = 0x80;

    uint8_t const ANS_SYNC_BYTE1 = 0xA5;
    uint8_t const ANS_SYNC_BYTE2 = 0x5A;

    uint8_t const RESP_MEAS_CHECKBIT = (0x1<<0);
    uint8_t const RESP_MEAS_SYNCBIT = (0x1<<0);
    uint8_t const RESP_MEAS_QUALITY_SHIFT = 2;
    uint8_t const RESP_MEAS_ANGLE_SHIFT = 1;

    uint8_t const ANS_TYPE_MEAS = 0x81;
    uint8_t const ANS_TYPE_DEV_INFO = 0x4;
    uint8_t const ANS_TYPE_DEV_HEALTH = 0x6;

    struct device_health_t {
      uint8_t   status;
      uint16_t  error_code;
    } __attribute__((packed));

    struct node_info_t {
      uint8_t    sync_quality; // syncbit:1; syncbit_inverse:1; quality:6;
      uint16_t   angle_q6_checkbit; // check_bit:1; angle_q6:15;
      uint16_t   distance_q2;
    } __attribute__((packed)) ;
 
    struct device_info_t {
      uint8_t   model;
      uint16_t  firmware_version;
      uint8_t   hardware_version;
      uint8_t   serialnum[16];
    } __attribute__((packed)) ;

    struct cmd_packet_t {
      uint8_t syncByte;
      uint8_t cmd_flag;
      uint8_t size;
      uint8_t data;
    } __attribute__((packed)) ;
    
    struct ans_header_t {
      uint8_t  syncByte1;
      uint8_t  syncByte2;
      uint32_t size:30;
      uint32_t subType:2;
      uint8_t  type;
    } __attribute__((packed));

  protected:
    bool motor_enabled;
    unsigned long int ring_start_ms[2];
    uint8_t recvPos = 0;
    node_info_t node;

  protected:
    virtual void enableMotor(bool enable);
    LDS::result_t getHealth(device_health_t & health, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    LDS::result_t getDeviceInfo(device_info_t & info, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    LDS::result_t abort();
    LDS::result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT_MS*2);
    virtual LDS::result_t waitScanDot();
    LDS::result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    LDS::result_t waitResponseHeader(ans_header_t * header, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    void markScanTime();
};
