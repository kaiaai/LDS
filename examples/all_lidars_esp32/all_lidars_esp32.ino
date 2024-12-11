// Copyright 2023-2024 KAIA.AI
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

#ifndef ESP32
  #error This example runs on ESP32
#endif

// 1. INCREASE Arduino IDE baud
const uint32_t SERIAL_MONITOR_BAUD = 115200;

// 2. CHANGE these to match your wiring
// IGNORE pins absent from your Lidar model (often EN, PWM)
const uint8_t LIDAR_GPIO_EN = 19; // ESP32 GPIO connected to Lidar EN pin
const uint8_t LIDAR_GPIO_RX = 16; // ESP32 GPIO connected to Lidar RX pin
const uint8_t LIDAR_GPIO_TX = 17; // ESP32 GPIO connected to Lidar TX pin
const uint8_t LIDAR_GPIO_PWM = 15;// ESP32 GPIO connected to Lidar PWM pin

// 3. UNCOMMENT your Lidar model
//
//#define NEATO_XV11
//#define SLAMTEC_RPLIDAR_A1
//#define XIAOMI_LDS02RR
//#define YDLIDAR_SCL
//#define YDLIDAR_X2_X2L
//#define YDLIDAR_X3
//#define YDLIDAR_X3_PRO
//#define 3IROBOTIX_DELTA_2G
//#define 3IROBOTIX_DELTA_2A_115200
//#define 3IROBOTIX_DELTA_2A
//#define 3IROBOTIX_DELTA_2B
#define LDROBOT_LD14P
//#define YDLIDAR_X4
//#define YDLIDAR_X4_PRO

// 4. Optional/Advanced
float LIDAR_SCAN_FREQ_TARGET = 5.0f; // Set desired rotations-per-second for some LiDAR models

// Usually you don't need to change the settings below
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 11;
const uint8_t LIDAR_PWM_CHANNEL = 0;
const uint16_t LIDAR_SERIAL_RX_BUF_LEN = 1024;

#include "lds_all_models.h"

HardwareSerial LidarSerial(1);
LDS *lidar;

void setupLidar() {
  #if ESP_IDF_VERSION_MAJOR >= 5
  if (!ledcAttachChannel(LIDAR_GPIO_PWM, LIDAR_PWM_FREQ,
    LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL))
    Serial.println("setupLidar() ledcAttachChannel() error");
  #else
  if (!ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS))
    Serial.println("setupLidar() ledcSetup() error");
  #endif

  #if defined(NEATO_XV11)
  lidar = new LDS_NEATO_XV11();
  #elif defined(SLAMTEC_RPLIDAR_A1)
  lidar = new LDS_RPLIDAR_A1();
  #elif defined(XIAOMI_LDS02RR)
  lidar = new LDS_LDS02RR();
  #elif defined(YDLIDAR_SCL)
  lidar = new LDS_YDLIDAR_SCL();
  #elif defined(YDLIDAR_X2_X2L)
  lidar = new LDS_YDLIDAR_X2_X2L();
  #elif defined(YDLIDAR_X3)
  lidar = new LDS_YDLIDAR_X3();
  #elif defined(YDLIDAR_X3_PRO)
  lidar = new LDS_YDLIDAR_X3_PRO();
  #elif defined(_3IROBOTIX_DELTA_2G)
  lidar = new LDS_DELTA_2G();
  #elif defined(_3IROBOTIX_DELTA_2A_115200)
  lidar = new LDS_DELTA_2A_115200();
  #elif defined(_3IROBOTIX_DELTA_2A)
  lidar = new LDS_DELTA_2A_230400();
  #elif defined(_3IROBOTIX_DELTA_2B)
  lidar = new LDS_DELTA_2B();
  #elif defined(LDROBOT_LD14P)
  lidar = new LDS_LDROBOT_LD14P();
  #elif defined(YDLIDAR_X4)
  lidar = new LDS_YDLIDAR_X4();
  #elif defined(YDLIDAR_X4_PRO)
  lidar = new LDS_YDLIDAR_X4_PRO();
  #else
    #error "Define a Lidar model"
  #endif

  lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);
  lidar->setInfoCallback(lidar_info_callback);
  lidar->setErrorCallback(lidar_error_callback);

  LidarSerial.begin(lidar->getSerialBaudRate(),
    SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);

  lidar->init();
  //lidar->stop();
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);

  setupLidar();

  Serial.print("LiDAR model ");
  Serial.println(lidar->getModelName());

  Serial.print("Lidar baud rate ");
  Serial.println(lidar->getSerialBaudRate());

  LDS::result_t result = lidar->start();
  Serial.print("startLidar() result: ");
  Serial.println(lidar->resultCodeToString(result));

  if (result < 0)
    Serial.println("Is the LiDAR connected to ESP32 and powered up?");

  lidar->setScanTargetFreqHz(LIDAR_SCAN_FREQ_TARGET);
}

int lidar_serial_read_callback() {
  return LidarSerial.read();
/*
  int ch = LidarSerial.read();
  if (ch != -1)
    Serial.println(ch);
  return ch;
*/
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {

  Serial.print(distance_mm);
  Serial.print(' ');
  Serial.println(angle_deg);

  if (scan_completed)
    Serial.println();
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lds_pin) {
/*
  Serial.print("Lidar pin ");
  Serial.print(lidar->pinIDToString(lds_pin));
  Serial.print(" set ");
  if (lds_pin > 0)
    Serial.print(value); // PWM value
  else
    Serial.print(lidar->pinStateToString((LDS::lds_pin_state_t)value));
  Serial.print(", RPM ");
  Serial.println(lidar->getCurrentScanFreqHz());
*/
  int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ?
    LIDAR_GPIO_EN : LIDAR_GPIO_PWM;

  if (int(value) <= LDS::DIR_INPUT) {

    // Configure pin direction
    if (int(value) == LDS::DIR_OUTPUT_PWM) {
      #if ESP_IDF_VERSION_MAJOR >= 5
      if (!ledcAttachChannel(pin, LIDAR_PWM_FREQ,
        LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL))
        Serial.println("lidar_motor_pin_callback() ledcAttachChannel() error");
      #else
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
      #endif
    } else
      pinMode(pin, (int(value) == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (int(value) < LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (int(value) == LDS::VALUE_HIGH) ? HIGH : LOW);
  else {
    // set PWM duty cycle
    int pwm_value = ((1<<LIDAR_PWM_BITS)-1)*value;
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
  }
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("LiDAR info ");
  Serial.print(lidar->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LiDAR error ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  return;
}

void loop() {
  lidar->loop();
}
