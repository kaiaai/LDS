// Copyright (c) 2024 kaia.ai, makerspet.com
// Apache 2.0 license
#pragma once
#include <Arduino.h>

// Interface to be implemented
class LDS {
  public:
    typedef int8_t result_t;
    typedef int8_t info_t;

    typedef void (*ScanPointCallback)(float, float);
    typedef void (*PacketCallback)(uint8_t*, uint16_t, bool);
    typedef void (*MotorPinCallback)(float, uint8_t);
    typedef size_t (*SerialWriteCallback)(const uint8_t *, size_t);
    typedef int (*SerialReadCallback)(void);
    typedef void (*InfoCallback)(info_t, String);
    typedef void (*ErrorCallback)(result_t, String);

    enum result_enum {
      OK = 0,
      ERROR_TIMEOUT,
      ERROR_INVALID_PACKET,
      ERROR_CRC,
      ERROR_NOT_READY,
      ERROR_NOT_IMPLEMENTED,
      ERROR_NOT_CONFIGURED,
      ERROR_MOTOR_DISABLED,
      ERROR_INVALID_MODEL,
      ERROR_DEVICE_INFO,
      ERROR_DEVICE_HEALTH,
      ERROR_START_SCAN,
      ERROR_INVALID_VALUE,
      ERROR_UNKNOWN,
    };
    static const int8_t DEFAULT_VALUE = -1;
    enum lds_pins {
      LDS_EN,
      LDS_MOTOR_PWM,
    };
    enum info_enum {
      INFO_MODEL,
      INFO_FIRMWARE_VERSION,
      INFO_HARDWARE_VERSION,
      INFO_SERIAL_NUMBER,
      INFO_DEVICE_HEALTH,
      INFO_SAMPLING_RATE,
      INFO_DEFAULT_TARGET_SCAN_FREQ_HZ,
      INFO_OTHER,
    };

  public:
    LDS();
    result_t start(); // Initialize and start; call from Arduino setup()
    void stop(); // Stop the motor and scanning
    void loop(); // Call frequently from Arduino loop()

    uint32_t getSerialBaudRate();
    float getCurrentScanFreq();
    float getTargetScanFreqHz();
    int getSamplingRateHz();
    bool isActive();

    result_t setScanTargetFreqHz(float freq);
    result_t setScanPIDCoeffs(float Kp, float Ki, float Kd);
    result_t setScanPIDSamplePeriodMs(uint32_t sample_period_ms);

    void setScanPointCallback(ScanPointCallback scan_callback);
    void setMotorPinCallback(MotorPinCallback motor_pin_callback);
    void setPacketCallback(PacketCallback packet_callback);
    void setSerialReadCallback(SerialReadCallback serial_read_callback);
    void setSerialWriteCallback(SerialWriteCallback serial_write_callback);
    void setInfoCallback(InfoCallback info_callback);
    void setErrorCallback(ErrorCallback error_callback);

    String resultCodeToString(result_t code);
    String infoCodeToString(info_t code);
    
  protected:
    void postScanPoint(float angle_deg, float dist_mm);
    void setMotorPin(float value, uint8_t pin);
    void postPacket(uint8_t* data, uint16_t length, bool scan_completed);
    int readSerial();
    size_t writeSerial(const uint8_t * buffer, size_t length);
    void postInfo(info_t code, String info);
    void postError(result_t code, String aux_info);

    void enableMotor(bool enable);

  protected:
    ScanPointCallback scan_point_callback;
    PacketCallback packet_callback;
    SerialWriteCallback serial_write_callback;
    SerialReadCallback serial_read_callback;
    MotorPinCallback motor_pin_callback;
    InfoCallback info_callback;
    ErrorCallback error_callback;
};
