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
// Based on https://github.com/EAIBOT/ydlidar_arduino

#pragma once
#include "LDS.h"

class LDS_YDLIDAR_X4 : public LDS {
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
    bool motor_enabled;
    unsigned long int ring_start_ms[2];

  protected:
    static const uint32_t DEFAULT_TIMEOUT_MS = 500;
    static const uint8_t PACKAGE_SAMPLE_MAX_LENGTH = 40;

    typedef enum {
      CT_NORMAL = 0,
      CT_RING_START = 1,
      CT_TAIL,
    } CT;
    
    struct node_info_t {
      uint8_t    sync_quality;
      uint16_t   angle_q6_checkbit;
      uint16_t   distance_q2;
    } __attribute__((packed)) ;
    
    struct device_info_t{
      uint8_t   model;
      uint16_t  firmware_version;
      uint8_t   hardware_version;
      uint8_t   serialnum[16];
    } __attribute__((packed)) ;
    
    struct device_health_t {
      uint8_t   status;
      uint16_t  error_code;
    } __attribute__((packed))  ;
    
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
    
    struct node_package_t {
      uint16_t  package_Head;
      uint8_t   package_CT; // package type
      uint8_t   nowPackageNum; // distance sample count
      uint16_t  packageFirstSampleAngle;
      uint16_t  packageLastSampleAngle;
      uint16_t  checkSum;
      uint16_t  packageSampleDistance[PACKAGE_SAMPLE_MAX_LENGTH]; // SCL hack
    } __attribute__((packed)) ;

  protected:
    virtual void enableMotor(bool enable);
    LDS::result_t getHealth(device_health_t & health, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    LDS::result_t getDeviceInfo(device_info_t & info, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    LDS::result_t abort(); // stop scanPoint op
    LDS::result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT_MS*2); // start scanPoint op
    virtual LDS::result_t waitScanDot(); // wait for one sample package to arrive
    LDS::result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    LDS::result_t waitResponseHeader(ans_header_t * header, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    void markScanTime();
    void checkInfo(int currentByte);

  protected:
    // Scan start packet: 2 bytes
    // Samples: 16 packets, up to 90B each total
    //   10 bytes header + 40*2=70 bytes samples
    // At 7Hz max total 1,442 bytes per scan
    static const uint8_t CMD_STOP = 0x65;
    static const uint8_t CMD_SCAN = 0x60;
    static const uint8_t CMD_FORCE_SCAN = 0x61;
    //static const uint8_t CMD_RESET = 0x80;
    static const uint8_t CMD_FORCE_STOP = 0x00;
    //static const uint8_t CMD_GET_EAI = 0x55;
    static const uint8_t CMD_GET_DEV_INFO = 0x90;
    static const uint8_t CMD_GET_DEV_HEALTH = 0x92;
    static const uint8_t CMD_SYNC_BYTE = 0xA5;
    static const uint16_t CMDFLAG_HAS_PAYLOAD = 0x8000;

    static const uint8_t ANS_TYPE_DEV_INFO = 0x4;
    static const uint8_t ANS_TYPE_DEV_HEALTH = 0x6;
    static const uint8_t ANS_SYNC_BYTE1 = 0xA5;
    static const uint8_t ANS_SYNC_BYTE2 = 0x5A;
    static const uint8_t ANS_TYPE_MEAS = 0x81;

    static const uint8_t RESP_MEAS_SYNCBIT = (0x1<<0);
    //static const uint8_t RESP_ MEAS_QUALITY_SHIFT = 2;
    static const uint8_t RESP_MEAS_CHECKBIT = (0x1<<0);
    static const uint8_t RESP_MEAS_ANGLE_SHIFT = 1;
    static const uint8_t RESP_MEAS_ANGLE_SAMPLE_SHIFT = 8;
    
    static const uint8_t PACKAGE_SAMPLE_BYTES = 2;
    static const uint16_t NODE_DEFAULT_QUALITY = 10; // (10<<2)
    static const uint8_t NODE_SYNC = 0x01;
    static const uint8_t NODE_NOT_SYNC = 2;
    static const uint8_t PACKAGE_PAID_BYTES = 10;
    static const uint16_t PH = 0x55AA; // Packet Header

    int recvPos = 0;
    uint8_t package_Sample_Num = 0;
    int package_recvPos = 0;
    int package_sample_sum = 0;

    node_package_t package;

    uint16_t package_Sample_Index = 0;
    float IntervalSampleAngle = 0;
    float IntervalSampleAngle_LastPackage = 0;
    uint16_t FirstSampleAngle = 0;
    uint16_t LastSampleAngle = 0;
    uint16_t CheckSum = 0;

    uint16_t CheckSumCal = 0;
    uint16_t SampleNumlAndCTCal = 0;
    uint16_t LastSampleAngleCal = 0;
    bool CheckSumResult = true;
    uint16_t Valu8Tou16 = 0;

    uint8_t state = 0;

    float scan_freq_hz = 0;
    bool scan_completed = false;
};
