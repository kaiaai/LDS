/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <lds_ydlidar_x4.h>
#include <lds_util.h>

#define LDS_EN_PIN 19 // ESP32 Dev Kit LDS enable pin
#define LDS_MOTOR_PWM_PIN 15 // LDS motor speed control using PWM
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

void lds_motor_pin_callback(float value, uint8_t pin) {
  if (pin == LDS::LDS_EN) {
    bool en = (value > 0);
    digitalWrite(LDS_EN_PIN, en);
    Serial.print(F("LDS "));
    Serial.println(en ? F("enabled") : F("disabled"));
  }
}

void loop() {
  lds.loop();
}
