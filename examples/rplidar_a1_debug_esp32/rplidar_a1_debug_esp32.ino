// Copyright 2023-2025 KAIA.AI
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

// 1. CHANGE these to match your wiring
// IGNORE pins absent from your Lidar model (often EN, PWM)
const uint8_t LIDAR_GPIO_EN = 19; // ESP32 GPIO connected to Lidar EN pin
//const uint8_t LIDAR_GPIO_RX = 16; // ESP32 GPIO connected to Lidar RX pin
//const uint8_t LIDAR_GPIO_TX = 17; // ESP32 GPIO connected to Lidar TX pin
//const uint8_t LIDAR_GPIO_PWM = 15;// ESP32 GPIO connected to Lidar PWM pin
const uint8_t LIDAR_GPIO_RX = 27; // ESP32 GPIO connected to Lidar RX pin
const uint8_t LIDAR_GPIO_TX = 35; // ESP32 GPIO connected to Lidar TX pin
const uint8_t LIDAR_GPIO_PWM = 13;// ESP32 GPIO connected to Lidar PWM pin

// 2. UNCOMMENT if using PWM pin and PWM LOW enables the motor
//#define INVERT_PWM_PIN

// 3. UNCOMMENT your Lidar model
//
//#define NEATO_XV11
#define SLAMTEC_RPLIDAR_A1
//#define XIAOMI_LDS02RR
//#define YDLIDAR_SCL
//#define YDLIDAR_X2_X2L
//#define YDLIDAR_X3
//#define YDLIDAR_X3_PRO
//#define _3IROBOTIX_DELTA_2G
//#define _3IROBOTIX_DELTA_2A_115200
//#define _3IROBOTIX_DELTA_2A
//#define _3IROBOTIX_DELTA_2B
//#define LDROBOT_LD14P
//#define YDLIDAR_X4
//#define YDLIDAR_X4_PRO
//#define CAMSENSE_X1

// 4. UNCOMMENT debug option(s)
// and increase SERIAL_MONITOR_BAUD to MAX possible
#define DEBUG_GPIO
//#define DEBUG_PACKETS
//#define DEBUG_SERIAL_IN
#define DEBUG_SERIAL_OUT

const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 11;
const uint8_t LIDAR_PWM_CHANNEL = 2;
const uint16_t LIDAR_SERIAL_RX_BUF_LEN = 1024;
const uint16_t PRINT_EVERY_NTH_POINT = 20;
const uint16_t HEX_DUMP_WIDTH = 16;

#include "lds_all_models.h"

HardwareSerial LidarSerial(1);
LDS *lidar;
uint16_t hex_dump_pos = 0;

void setupLidar() {

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
  #elif defined(CAMSENSE_X1)
  lidar = new LDS_CAMSENSE_X1();
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

  delay(200);

  LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);

  delay(200);

  lidar->init();
  //lidar->stop();
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);

  Serial.println();
  Serial.print("ESP IDF version ");
  Serial.println(ESP_IDF_VERSION_MAJOR);

  setupLidar();

  delay(200);

  Serial.print("LiDAR model ");
  Serial.print(lidar->getModelName());
  Serial.print(", baud rate ");
  Serial.print(lidar->getSerialBaudRate());
  Serial.print(", TX GPIO ");
  Serial.print(LIDAR_GPIO_TX);
  Serial.print(", RX GPIO ");
  Serial.println(LIDAR_GPIO_RX);

  LDS::result_t result = lidar->start();
  Serial.print("startLidar() result: ");
  Serial.println(lidar->resultCodeToString(result));

  // Set desired rotations-per-second for some LiDAR models
  // lidar->setScanTargetFreqHz(5.0f);
}

void printByteAsHex(uint8_t b) {
  if (b < 16)
    Serial.print('0');
  Serial.print(b, HEX);
  Serial.print(' ');
}

void printBytesAsHex(const uint8_t * buffer, uint16_t length) {
  if (length == 0)
    return;

  for (uint16_t i = 0; i < length; i++) {
    printByteAsHex(buffer[i]);
  }
}

int lidar_serial_read_callback() {
  #ifdef DEBUG_SERIAL_IN
  int ch = LidarSerial.read();
  if (ch != -1) {
    if (hex_dump_pos++ % HEX_DUMP_WIDTH == 0)
      Serial.println();
    printByteAsHex(ch);
  }
  return ch;
  #else
  return LidarSerial.read();
  #endif
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
  #ifdef DEBUG_SERIAL_OUT
  Serial.println();
  Serial.print('>');
  printBytesAsHex(buffer, length);
  Serial.println();
  #endif
  
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  static int i=0;
  
  if (scan_completed) {
    i = 0;
    Serial.print("Scan completed; scans-per-second ");
    Serial.println(lidar->getCurrentScanFreqHz());
  }

  if (i % PRINT_EVERY_NTH_POINT == 0) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(distance_mm);
    Serial.print(' ');
    Serial.println(angle_deg);
  }
  i++;
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {

  int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_GPIO_EN : LIDAR_GPIO_PWM;

  #ifdef DEBUG_GPIO
  Serial.print("GPIO ");
  Serial.print(pin);
  Serial.print(' ');
  Serial.print(lidar->pinIDToString(lidar_pin));
  Serial.print(" mode set to ");
  Serial.println(lidar->pinStateToString((LDS::lds_pin_state_t) int(value)));
  #endif

  if (value <= (float)LDS::DIR_INPUT) {

    // Configure pin direction
    if (value == (float)LDS::DIR_OUTPUT_PWM) {

      #if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
      #else
      if (!ledcAttachChannel(pin, LIDAR_PWM_FREQ, LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL))
        Serial.println("lidar_motor_pin_callback() ledcAttachChannel() error");
      #endif
    } else
      pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);

    return;
  }

  if (value < (float)LDS::VALUE_PWM) {
    // Set constant output
    // TODO invert PWM as needed
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);

  } else {
    #ifdef DEBUG_GPIO
    Serial.print("PWM value set to ");
    Serial.print(value);
    #endif

    // set PWM duty cycle
    #ifdef INVERT_PWM_PIN
    value = 1 - value;
    #endif

    int pwm_value = ((1<<LIDAR_PWM_BITS)-1)*value;

    #if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
    #else
    ledcWriteChannel(LIDAR_PWM_CHANNEL, pwm_value);
    #endif

    #ifdef DEBUG_GPIO
    Serial.print(' ');
    Serial.println(pwm_value);
    #endif
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
  #ifdef DEBUG_PACKETS
  Serial.println();
  Serial.print("Packet callback, length=");
  Serial.print(length);
  Serial.print(", scan_completed=");
  Serial.println(scan_completed);
  #endif
  
  return;
}

void loop() {
  lidar->loop();
}
