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

#include <lds_ydlidar_x4.h>
#include <lds_util.h>

#define LDS_MOTOR_EN_PIN 19 // ESP32 Dev Kit LDS enable pin
#define LDS_MOTOR_PWM_PIN 15 // LDS motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ    10000
#define LDS_MOTOR_PWM_BITS    11
#define LDS_MOTOR_PWM_CHANNEL    2 // ESP32 PWM channel for LDS motor speed control

HardwareSerial LdSerial(2); // TX 17, RX 16
LDS_YDLidarX4 lds;

void setup() {
  pinMode(LDS_MOTOR_PWM_PIN, INPUT); // Leave unused
  pinMode(LDS_EN_PIN, OUTPUT);

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

void lds_scan_point_callback(float angle_deg, float distance_mm) {
  static int i=0;

  if (i++ % 20 == 0) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(distance_mm);
    Serial.print(' ');
    Serial.println(angle_deg);
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

void lds_motor_pin_callback(float value, LDS::lds_pin lds_pin) {
  int pin = (lds_pin == LDS::LDS_MOTOR_EN) ? LDS_MOTOR_EN_PIN : LDS_MOTOR_PWM_PIN;

  if (value <= LDS::INPUT) {
    // Configure pin direction
    if (value == LDS::OUTPUT_PWM) {
      ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ, LDS_MOTOR_PWM_BITS);
      ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::PWM) // set constant output
    digitalWrite(pin, (value == LDS::HIGH) ? HIGH : LOW);
  else { // set PWM duty cycle
    int pwm_value = ((1<<LDS_MOTOR_PWM_BITS)-1)*value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}


void loop() {
  lds.loop();
}
