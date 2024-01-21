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

#ifndef ESP32
  #error This example runs on ESP32
#endif

#include <LDS_YDLIDAR_X4.h>

const uint8_t LDS_MOTOR_EN_PIN = 19; // ESP32 Dev Kit LDS enable pin
const uint8_t LDS_MOTOR_PWM_PIN = 15; // LDS motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ    10000
#define LDS_MOTOR_PWM_BITS    11
#define LDS_MOTOR_PWM_CHANNEL    2 // ESP32 PWM channel for LDS motor speed control

HardwareSerial LdSerial(2); // TX 17, RX 16
LDS_YDLIDAR_X4 lds;

void setup() {
  lds.setScanPointCallback(lds_scan_point_callback);
  lds.setPacketCallback(lds_packet_callback);
  lds.setSerialWriteCallback(lds_serial_write_callback);
  lds.setSerialReadCallback(lds_serial_read_callback);
  lds.setMotorPinCallback(lds_motor_pin_callback);

  Serial.println(LdSerial.setRxBufferSize(1024)); // must be before .begin()
  Serial.print("LDS RX buffer size "); // default 128 hw + 256 sw
  uint32_t baud_rate = lds.getSerialBaudRate();
  Serial.print("LDS baud rate ");
  Serial.print(baud_rate);

  LdSerial.begin(baud_rate);
  while (LdSerial.read() >= 0);

  LDS::result_t result = lds.start();
  Serial.print("LDS init() result: ");
  Serial.println(lds.resultCodeToString(result));

  if (result < 0)
    Serial.println("WARNING: is LDS connected to ESP32?");

  lds.start();
}

int lds_serial_read_callback() {
  return LdSerial.read();
}

size_t lds_serial_write_callback(const uint8_t * buffer, size_t length) {
  return LdSerial.write(buffer, length);
}

void lds_scan_point_callback(float angle_deg, float distance_mm, float quality,
  bool scan_completed) {
  static int i=0;

  if ((i++ % 20 == 0) || scan_completed) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(distance_mm);
    Serial.print(' ');
    Serial.print(angle_deg);
    if (scan_completed)
      Serial.println('*');
    else
      Serial.println();
  }
}

void lds_info_callback(LDS::info_t code, String info) {
  Serial.print("LDS info ");
  Serial.print(lds.infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lds_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LDS error ");
  Serial.print(lds.resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lds_motor_pin_callback(float value, LDS::lds_pin_t lds_pin) {
  int pin = (lds_pin == LDS::LDS_MOTOR_EN_PIN) ?
    LDS_MOTOR_EN_PIN : LDS_MOTOR_PWM_PIN;

  if (value <= LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ, LDS_MOTOR_PWM_BITS);
      ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::VALUE_PWM) // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<LDS_MOTOR_PWM_BITS)-1)*value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

void lds_packet_callback(uint8_t * packet, uint16_t length, bool scan_completed) {
  return;
}

void loop() {
  lds.loop();
}
