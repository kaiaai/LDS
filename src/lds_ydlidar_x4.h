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
// Based on
//   Copyright 2015 - 2018 EAI TEAM http://www.eaibot.com
//   https://github.com/EAIBOT/ydlidar_arduino

#pragma once
#include "lds.h"

class LDS_YDLidarX4 : public LDS {
  public:
    LDS_YDLidarX4();

    result_t start();
    void stop();
    void loop(); // Call loop() frequently from Arduino loop()

    uint32_t getSerialBaudRate();
    float getCurrentScanFreqHz();
    float getTargetScanFreqHz();
    int getSamplingRateHz();
    bool isActive();

    result_t setScanTargetFreqHz(float freq);
    result_t setScanPIDCoeffs(float Kp, float Ki, float Kd);
    result_t setScanPIDSamplePeriodMs(uint32_t sample_period_ms);

  protected:
    bool motor_enabled;
    float target_scan_freq;
    int sampling_rate;

  protected:
    static const uint32_t DEFAULT_TIMEOUT_MS = 500;
    static const uint8_t PACKAGE_SAMPLE_MAX_LENGTH = 0x80;

    typedef enum {
      CT_NORMAL = 0,
      CT_RING_START  = 1,
      CT_TAIL,
    } CT;
    
    struct node_info {
      uint8_t    sync_quality;
      uint16_t   angle_q6_checkbit;
      uint16_t   distance_q2;
    } __attribute__((packed)) ;
    
    struct device_info{
      uint8_t   model;
      uint16_t  firmware_version;
      uint8_t   hardware_version;
      uint8_t   serialnum[16];
    } __attribute__((packed)) ;
    
    struct device_health {
      uint8_t   status;
      uint16_t  error_code;
    } __attribute__((packed))  ;
    
    struct cmd_packet {
      uint8_t syncByte;
      uint8_t cmd_flag;
      uint8_t size;
      uint8_t data;
    } __attribute__((packed)) ;
    
    struct lidar_ans_header {
      uint8_t  syncByte1;
      uint8_t  syncByte2;
      uint32_t size:30;
      uint32_t subType:2;
      uint8_t  type;
    } __attribute__((packed));
    
    struct node_package {
      uint16_t  package_Head;
      uint8_t   package_CT;
      uint8_t   nowPackageNum;
      uint16_t  packageFirstSampleAngle;
      uint16_t  packageLastSampleAngle;
      uint16_t  checkSum;
      uint16_t  packageSampleDistance[PACKAGE_SAMPLE_MAX_LENGTH];
    } __attribute__((packed)) ;

  protected:
    void enableMotor(bool enable);
    LDS::result_t getHealth(device_health & health, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    LDS::result_t getDeviceInfo(device_info & info, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    LDS::result_t abort(); // stop scanPoint op
    LDS::result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT_MS*2); // start scanPoint op
    LDS::result_t waitScanDot(); // wait for one sample package to arrive
    LDS::result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);
    LDS::result_t waitResponseHeader(lidar_ans_header * header, uint32_t timeout = DEFAULT_TIMEOUT_MS);
    void setupPins();

  protected:
    static const uint8_t LIDAR_CMD_STOP = 0x65;
    static const uint8_t LIDAR_CMD_SCAN = 0x60;
    static const uint8_t LIDAR_CMD_FORCE_SCAN = 0x61;
    static const uint8_t LIDAR_CMD_RESET = 0x80;
    static const uint8_t LIDAR_CMD_FORCE_STOP = 0x00;
    static const uint8_t LIDAR_CMD_GET_EAI = 0x55;
    static const uint8_t LIDAR_CMD_GET_DEVICE_INFO = 0x90;
    static const uint8_t LIDAR_CMD_GET_DEVICE_HEALTH = 0x92;
    static const uint8_t LIDAR_CMD_SYNC_BYTE = 0xA5;
    static const uint16_t LIDAR_CMDFLAG_HAS_PAYLOAD = 0x8000;

    static const uint8_t LIDAR_ANS_TYPE_DEVINFO = 0x4;
    static const uint8_t LIDAR_ANS_TYPE_DEVHEALTH = 0x6;
    static const uint8_t LIDAR_ANS_SYNC_BYTE1 = 0xA5;
    static const uint8_t LIDAR_ANS_SYNC_BYTE2 = 0x5A;
    static const uint8_t LIDAR_ANS_TYPE_MEASUREMENT = 0x81;

    static const uint8_t LIDAR_RESP_MEASUREMENT_SYNCBIT = (0x1<<0);
    static const uint8_t LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT = 2;
    static const uint8_t LIDAR_RESP_MEASUREMENT_CHECKBIT = (0x1<<0);
    static const uint8_t LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT = 1;
    static const uint8_t LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT = 8;
    
    static const uint8_t LIDAR_CMD_RUN_POSITIVE = 0x06;
    static const uint8_t LIDAR_CMD_RUN_INVERSION = 0x07;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_ADDMIC = 0x09;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_DISMIC = 0x0A;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_ADD = 0x0B;
    static const uint8_t LIDAR_CMD_SET_AIMSPEED_DIS = 0x0C;
    static const uint8_t LIDAR_CMD_GET_AIMSPEED = 0x0D;
    static const uint8_t LIDAR_CMD_SET_SAMPLING_RATE = 0xD0;
    static const uint8_t LIDAR_CMD_GET_SAMPLING_RATE = 0xD1;
    
    static const uint8_t LIDAR_STATUS_OK = 0x0;
    static const uint8_t LIDAR_STATUS_WARNING = 0x1;
    static const uint8_t LIDAR_STATUS_ERROR = 0x2;
    
    static const uint8_t PACKAGE_SAMPLE_BYTES = 2;
    static const uint16_t NODE_DEFAULT_QUALITY = (10<<2);
    static const uint8_t NODE_SYNC = 1;
    static const uint8_t NODE_NOT_SYNC = 2;
    static const uint8_t PACKAGE_PAID_BYTES = 10;
    static const uint16_t PH = 0x55AA;
};
