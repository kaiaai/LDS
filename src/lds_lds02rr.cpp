// Based on
//   Copyright 2014-2021 James LeRoy getSurreal.com
//   https://github.com/getSurreal/XV_Lidar_Controller
//
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

#include "lds_lds02rr.h"

LDS_LDSRR02::LDS_LDSRR02() : LDS() {
  motor_enabled = false;
  clearVars();

  pwm_val = 0.5;

  scan_rpm_setpoint = DEFAULT_SCAN_RPM;  // desired RPM 1.8KHz/5FPS/360 = 1 deg resolution
  scanFreqPID.init(&scan_rpm, &pwm_val, &scan_rpm_setpoint, 3.0e-3, 1.0e-3, 0.0, PID_v1::DIRECT);
  scanFreqPID.SetOutputLimits(0, 1.0);
  scanFreqPID.SetSampleTime(20);
  scanFreqPID.SetMode(PID_v1::AUTOMATIC);

  scan_rpm = 0;
}

uint32_t LDS_LDSRR02::getSerialBaudRate() {
  return 115200;
}

int LDS_LDSRR02::getSamplingRateHz() {
  return 1800;
}
LDS::result_t LDS_LDSRR02::setScanPIDCoeffs(float Kp, float Ki, float Kd) {
  scanFreqPID.SetTunings(Kp, Ki, Kd);
}

LDS::result_t LDS_LDSRR02::setScanPIDSamplePeriodMs(uint32_t sample_period_ms) {
  scanFreqPID.SetSampleTime(sample_period_ms);
  return LDS::LDS::RESULT_OK;
}

void LDS_LDSRR02::loop() {
  LDS::result_t result = LDS::LDS::RESULT_OK;
  
  while (true) {
    int c = readSerial();
    if (c < 0)
      break;

    result_t result = processByte(c);
    if (result < 0)
      postError(result, "processByte()");
  }

  if (!motor_enabled)
    return;

  scanFreqPID.Compute();

  if (pwm_val != pwm_last) {
    setMotorPin(pwm_val, LDS_MOTOR_PWM_PIN);
    pwm_last = pwm_val;
  }
}

bool LDS_LDSRR02::isActive() {
  return motor_enabled;
}

float LDS_LDSRR02::getCurrentScanFreqHz() {
  return scan_rpm/60.0f;
}

void LDS_LDSRR02::clearVars() {
  for (int ix = 0; ix < N_DATA_QUADS; ix++) {
    aryDist[ix] = 0;
    aryQuality[ix] = 0;
    //aryInvalidDataFlag[ix] = 0;
  }
  for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
    Packet[ixPacket] = 0;
  ixPacket = 0;
  eState = eState_Find_COMMAND; // This packet is done -- loLDS::RESULT_OK for next COMMAND byte
}

bool LDS_LDSRR02::isValidPacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[OFFSET_TO_CRC_M];

  return ((b1a == b1b) && (b2a == b2b));
}

void LDS_LDSRR02::enableMotor(bool enable) {
  motor_enabled = enable;

  if (enable) {
    setMotorPin(DIR_OUTPUT_PWM, LDS_MOTOR_PWM_PIN);
    setMotorPin(enable ? pwm_val : VALUE_LOW, LDS_MOTOR_PWM_PIN);
  } else {
    setMotorPin(DIR_OUTPUT_CONST, LDS_MOTOR_PWM_PIN);
    setMotorPin(VALUE_LOW, LDS_MOTOR_PWM_PIN);
  }
}

// TODO uint8 iQuad?
void LDS_LDSRR02::processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

byte LDS_LDSRR02::processDistance(int iQuad) {
  // Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
  //   byte 0 : <distance 7:0>
  //   byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
  //   byte 2 : <signal strength 7:0>
  //   byte 3 : <signal strength 15:8>
  // dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
  // so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // LDS::RESULT_OKay
}

void LDS_LDSRR02::processSpeed() {
  // Extract motor speed from packet - two bytes little-endian, equals RPM/64
  uint8_t scan_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  uint8_t scan_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  scan_rpm = float( (scan_rph_high_byte << 8) | scan_rph_low_byte ) / 64.0;
}

uint16_t LDS_LDSRR02::processIndex() {
  // processIndex - Process the packet element 'index'
  // index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
  //    (packet 89, readings 356 to 359).
  // Returns the first angle (of 4) in the current 'index' group
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4

  return angle;
}

LDS::result_t LDS_LDSRR02::processByte(int inByte) {
  // Switch, based on 'eState':
  // State 1: We're scanning for 0xFA (COMMAND) in the input stream
  // State 2: Build a complete data packet
  LDS::result_t result = LDS::LDS::RESULT_OK;
  if (eState == eState_Find_COMMAND) {      // flush input until we get COMMAND byte
    if (inByte == COMMAND) {
      eState++;                                 // switch to 'build a packet' state
      Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
    }
  } else {
    Packet[ixPacket++] = inByte;        // keep storing input into 'Packet'
    if (ixPacket == PACKET_LENGTH) {
      // we've got all the input bytes, so we're done building this packet
      
      if (isValidPacket()) {      // Check packet CRC
        byte aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set

        // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)
        // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
        uint16_t startingAngle = processIndex();

        postPacket(Packet, PACKET_LENGTH, UNKNOWN_POS); // TODO

        processSpeed();

        // process each of the (4) sets of data in the packet
        for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
          aryInvalidDataFlag[ix] = processDistance(ix);
        for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
          aryQuality[ix] = 0;
          if (aryInvalidDataFlag[ix] == 0)
            processSignalStrength(ix);
        }

        for (int ix = 0; ix < N_DATA_QUADS; ix++) {
          byte err = aryInvalidDataFlag[ix] & BAD_DATA_MASK;

          if (!err)
            postScanPoint(startingAngle + ix, aryDist[ix]); // aryQuality[ix], err
        }

      } else {
        // Bad packet
        result = ERROR_CRC;
      }

      clearVars();   // initialize a bunch of stuff before we switch back to State 1
    }
  }
  return result;
}

LDS::result_t LDS_LDSRR02::setScanTargetFreqHz(float freq) {
  float rpm = freq * 60.0f;
  if (rpm <= 0) {
    scan_rpm_setpoint = DEFAULT_SCAN_RPM;
    return LDS::LDS::RESULT_OK;
  }
  
  if (rpm <= DEFAULT_SCAN_RPM*0.9f || rpm >= DEFAULT_SCAN_RPM*1.1f)
    return ERROR_INVALID_VALUE;

  scan_rpm_setpoint = rpm;
  return LDS::LDS::RESULT_OK;
}

float LDS_LDSRR02::getTargetScanFreqHz() {
  return scan_rpm_setpoint / 60.0f;
}

void LDS_LDSRR02::stop() {
  enableMotor(false);
}

LDS::result_t LDS_LDSRR02::start() {
  enableMotor(true);
  postInfo(INFO_SAMPLING_RATE, String(getSamplingRateHz()));
  postInfo(INFO_DEFAULT_TARGET_SCAN_FREQ_HZ, String(DEFAULT_SCAN_RPM/60.0f));
}