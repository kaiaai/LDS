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

#define YDLIDAR_X4_PRO_MODEL_NUM 4

class LDS_YDLIDAR_X4_PRO : public LDS {
  public:
    virtual void init() override;

    virtual result_t start() override;
    virtual result_t stop() override;
    virtual void loop() override; // Call loop() frequently from Arduino loop()

    virtual uint32_t getSerialBaudRate() override;
    virtual float getCurrentScanFreqHz() override;
    float getReportedScanFreqHz();
    virtual float getTargetScanFreqHz() override;
    virtual int getSamplingRateHz() override;
    virtual bool isActive() override;
    virtual const char* getModelName() override;

    virtual result_t setScanTargetFreqHz(float freq) override;

    struct device_health_t {
      bool sensor_issue;
      bool encoding_issue;
      bool wireless_power_issue;
      bool power_delivery_issue;
      bool laser_issue;
      bool data_issue;
    };
    LDS::result_t getHealth(device_health_t & health);


    struct device_info_t {
      uint8_t customer_version_major;
      uint8_t customer_version_minor;
      uint8_t hardware_version;
      uint8_t major_firmware_version;
      uint8_t minor_firmware_version;
      uint16_t manufacture_year;
      uint8_t manufacture_month;
      uint8_t manufacture_day;
      uint32_t serial_num;
    };
    LDS::result_t getDeviceInfo(device_info_t & info);

  protected:
    bool motor_enabled = false;
    unsigned long int ring_start_ms[2];
    bool power_on_info_processed = false;

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
      uint16_t   distance;
    } __attribute__((packed)) ;

    struct ans_header_t {
      uint8_t  syncByte1;
      uint8_t  syncByte2;
      uint32_t size:30;
      uint32_t subType:2;
      uint8_t  type;
    } __attribute__((packed));

    struct distance_measurement_t {
      uint8_t   packageInterference:2;
      uint8_t   packageDistanceLByte:6;
      uint8_t   packageDistanceHByte;
    } __attribute__((packed)) ;

    struct node_package_t {
      uint16_t  package_Head;
      uint8_t   package_CT; // package type
      uint8_t   nowPackageNum; // distance sample count
      uint16_t  packageFirstSampleAngle;
      uint16_t  packageLastSampleAngle;
      uint16_t  checkSum;
      distance_measurement_t packageSampleDistance[PACKAGE_SAMPLE_MAX_LENGTH];
    } __attribute__((packed)) ;

  protected:
    void initMotor();
    virtual void enableMotor(bool enable);
    LDS::result_t abort();
    LDS::result_t startScan();
    virtual LDS::result_t waitScanDot(); // wait for one sample package to arrive
    LDS::result_t waitMessageHeader(ans_header_t * header, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    void markScanTime();
    void checkInfo(int currentByte);

  protected:
    // Scan start packet: 2 bytes
    // Samples: 16 packets, up to 90B each total
    //   10 bytes header + 40*2=70 bytes samples
    // At 7Hz max total 1,442 bytes per scan
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
    uint8_t cached_ct_packets[14] = {0};
    bool ct_packets_ready = false;

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

    bool scan_completed = false;
    uint8_t packets_since_last_completed_scan = 0;
};
