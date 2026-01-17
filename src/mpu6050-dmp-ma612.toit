import binary
import log
import math show *

import serial.device as serial
import serial.registers as registers

import .mpu6050 show *

// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 6.12 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//  2021/09/27 - split implementations out of header files, finally
//  2019/07/10 - I incorporated DMP Firmware Version 6.12 Latest as of today with many features and bug fixes.
//             - MPU6050 Registers have not changed just the DMP Image so that full backwards compatibility is present
//             - Run-time calibration routine is enabled which calibrates after no motion state is detected
//             - once no motion state is detected Calibration completes within 0.5 seconds
//             - The Drawback is that the firmware image is larger.
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
/*
// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS612

#include "MPU6050_6Axis_MotionApps612.h"


/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

//#define DEBUG
#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
#endif

#define MPU6050_DMP_CODE_SIZE       3062    // dmpMemory[]
*/



// *** this is a capture of the DMP Firmware V6.1.2 after all the messy changes were made so we can just load it

class Mpu6050-dmp-ma612 extends Mpu6050:
  // Original Code:
  // https://github.com/jrowberg/i2cdevlib/raw/refs/heads/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps612.h
  // https://raw.githubusercontent.com/jrowberg/i2cdevlib/refs/heads/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps612.cpp

  static I2C-ADDRESS ::= Mpu6050.I2C-ADDRESS
  static I2C-ADDRESS-AD0-1 ::= Mpu6050.I2C-ADDRESS-AD0-1
  static MPU-6050-WHOAMI ::= Mpu6050.MPU-6050-WHOAMI

  static SLEEP-CYCLE-WAKE-FREQ-1-25HZ ::= Mpu6050.SLEEP-CYCLE-WAKE-FREQ-1-25HZ
  static SLEEP-CYCLE-WAKE-FREQ-5HZ    ::= Mpu6050.SLEEP-CYCLE-WAKE-FREQ-5HZ
  static SLEEP-CYCLE-WAKE-FREQ-20HZ   ::= Mpu6050.SLEEP-CYCLE-WAKE-FREQ-20HZ
  static SLEEP-CYCLE-WAKE-FREQ-40HZ   ::= Mpu6050.SLEEP-CYCLE-WAKE-FREQ-40HZ

  static CLOCK-SRC-INTERNAL-8MHZ    ::= Mpu6050.CLOCK-SRC-INTERNAL-8MHZ
  static CLOCK-SRC-PLL-X-G          ::= Mpu6050.CLOCK-SRC-PLL-X-G
  static CLOCK-SRC-PLL-Y-G          ::= Mpu6050.CLOCK-SRC-PLL-Y-G
  static CLOCK-SRC-PLL-Z-G          ::= Mpu6050.CLOCK-SRC-PLL-Z-G
  static CLOCK-SRC-PLL-EXT-32768KHZ ::= Mpu6050.CLOCK-SRC-PLL-EXT-32768KHZ
  static CLOCK-SRC-PLL-EXT-192MHZ   ::= Mpu6050.CLOCK-SRC-PLL-EXT-192MHZ
  static CLOCK-STOP                 ::= Mpu6050.CLOCK-STOP

  static CONFIG-DLPF-0 ::= Mpu6050.CONFIG-DLPF-0  // ACCEL= 260 Hz, 0 ms   / GYRO= 256 Hz, 0.98 ms / SR= 8 kHz
  static CONFIG-DLPF-1 ::= Mpu6050.CONFIG-DLPF-1  // ACCEL= 184 Hz, 2.0 ms / GYRO= 188 Hz, 1.9 ms  / SR= 1 kHz
  static CONFIG-DLPF-2 ::= Mpu6050.CONFIG-DLPF-2  // ACCEL= 94 Hz, 3.0 ms  / GYRO= 98 Hz, 2.8 1 ms / SR= 1 kHz
  static CONFIG-DLPF-3 ::= Mpu6050.CONFIG-DLPF-3  // ACCEL= 44 Hz, 4.9 ms  / GYRO= 42 Hz, 4.8 1 ms / SR= 1 kHz
  static CONFIG-DLPF-4 ::= Mpu6050.CONFIG-DLPF-4  // ACCEL= 21 Hz, 8.5 ms  / GYRO= 20 Hz, 8.3 1 ms / SR= 1 kHz
  static CONFIG-DLPF-5 ::= Mpu6050.CONFIG-DLPF-5  // ACCEL= 10 Hz, 13.8 ms / GYRO= 10 Hz, 13.4 ms  / SR= 1 kHz
  static CONFIG-DLPF-6 ::= Mpu6050.CONFIG-DLPF-6  // ACCEL= 5 Hz, 19.0 ms  / GYRO= 5 Hz, 18.6 1 ms / SR= 1 kHz

  static GYRO-FS-131-0 ::= Mpu6050.GYRO-FS-131-0  // 131 LSB/deg/s  +/- 250deg/s
  static GYRO-FS-65-5  ::= Mpu6050.GYRO-FS-65-5  // 65.5 LSB/deg/s  +/- 500deg/s
  static GYRO-FS-32-8  ::= Mpu6050.GYRO-FS-32-8  // 32.8 LSB/deg/s  +/- 1000deg/s
  static GYRO-FS-16-4  ::= Mpu6050.GYRO-FS-16-4  // 16.4 LSB/deg/s  +/- 2000deg/s

  static ACCEL-HPF-RESET  ::= Mpu6050.ACCEL-HPF-RESET // Reset
  static ACCEL-HPF-5HZ    ::= Mpu6050.ACCEL-HPF-5HZ // On @ 5 Hz
  static ACCEL-HPF-2-5HZ  ::= Mpu6050.ACCEL-HPF-2-5HZ // On @ 2.5 Hz
  static ACCEL-HPF-1-25HZ ::= Mpu6050.ACCEL-HPF-1-25HZ // On @ 1.25 Hz
  static ACCEL-HPF-0-63HZ ::= Mpu6050.ACCEL-HPF-0-63HZ // On @ 0.63 Hz
  static ACCEL-HPF-HOLD   ::= Mpu6050.ACCEL-HPF-HOLD // Hold
  static ACCEL-FS-RANGE-2G  ::= Mpu6050.ACCEL-FS-RANGE-2G  // 16384 LSB/g  +/- 2g
  static ACCEL-FS-RANGE-4G  ::= Mpu6050.ACCEL-FS-RANGE-4G  // 8192 LSB/g  +/- 4g
  static ACCEL-FS-RANGE-8G  ::= Mpu6050.ACCEL-FS-RANGE-8G  // 4096 LSB/g  +/- 8g
  static ACCEL-FS-RANGE-16G ::= Mpu6050.ACCEL-FS-RANGE-16G  // 2048 LSB/g  +/- 16g

  static MOT-DETECT-X-NEG ::= Mpu6050.MOT-DETECT-X-NEG
  static MOT-DETECT-X-POS ::= Mpu6050.MOT-DETECT-X-POS
  static MOT-DETECT-Y-NEG ::= Mpu6050.MOT-DETECT-Y-NEG
  static MOT-DETECT-Y-POS ::= Mpu6050.MOT-DETECT-Y-POS
  static MOT-DETECT-Z-NEG ::= Mpu6050.MOT-DETECT-Z-NEG
  static MOT-DETECT-Z-POS ::= Mpu6050.MOT-DETECT-Z-POS
  static MOT-DETECT-MOT-TO-ZMOT ::= Mpu6050.MOT-DETECT-MOT-TO-ZMOT

  static dmp-ma-612_/ByteArray ::= #[
    0x00, 0xF8, 0xF6, 0x2A, 0x3F, 0x68, 0xF5, 0x7A, 0x00, 0x06, 0xFF, 0xFE, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x03, 0x0C, 0x30, 0xC3, 0x0A, 0x74, 0x56, 0x2D, 0x0D, 0x62, 0xDB, 0xC7, 0x16, 0xF4, 0xBA, 0x02,
    0x38, 0x83, 0xF8, 0x83, 0x30, 0x00, 0xF8, 0x83, 0x25, 0x8E, 0xF8, 0x83, 0x30, 0x00, 0xF8, 0x83,
    0xFF, 0xFF, 0xFF, 0xFF, 0x0C, 0xBD, 0xD8, 0x11, 0x24, 0x00, 0x04, 0x00, 0x1A, 0x82, 0x79, 0xA1,
    0x00, 0x36, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6F, 0xA2,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x1F, 0xA4, 0xE8, 0xE4, 0xFF, 0xF5, 0xDC, 0xB9, 0x00, 0x5B, 0x79, 0xCF, 0x1F, 0x3F, 0x78, 0x76,
    0x00, 0x86, 0x7C, 0x5A, 0x00, 0x86, 0x23, 0x47, 0xFA, 0xB9, 0x86, 0x31, 0x00, 0x74, 0x87, 0x8A,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x43, 0x05, 0xFF, 0xFF, 0xE9, 0xA8, 0x00, 0x00, 0x21, 0x82,
    0xFA, 0xB8, 0x4D, 0x46, 0xFF, 0xFA, 0xDF, 0x3D, 0xFF, 0xFF, 0xB2, 0xB3, 0x00, 0x00, 0x00, 0x00,
    0x3F, 0xFF, 0xBA, 0x98, 0x00, 0x5D, 0xAC, 0x08, 0x00, 0x0A, 0x63, 0x78, 0x00, 0x01, 0x46, 0x21,
    0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x42, 0xB5, 0x00, 0x06, 0x00, 0x64, 0x00, 0x64, 0x00, 0x06,
    0x14, 0x06, 0x02, 0x9F, 0x0F, 0x47, 0x91, 0x32, 0xD9, 0x0E, 0x9F, 0xC9, 0x1D, 0xCF, 0x4C, 0x34,
    0x3B, 0xB6, 0x7A, 0xE8, 0x00, 0x64, 0x00, 0x06, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE,
    /* bank # 1 */
    0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x07, 0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0xFA, 0x46, 0x00, 0x00, 0xA2, 0xB8, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x04, 0xD6, 0x00, 0x00, 0x04, 0xCC, 0x00, 0x00, 0x04, 0xCC, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x32, 0xF8, 0x98, 0x00, 0x00, 0xFF, 0x65, 0x00, 0x00, 0x83, 0x0F, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0xFA, 0x46, 0x00, 0x00, 0xA2, 0xB8, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x01, 0xFB, 0x83, 0x00, 0x7C, 0x00, 0x00, 0xFB, 0x15, 0xFC, 0x00, 0x1F, 0xB4, 0xFF, 0x83,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x65, 0x00, 0x07, 0x00, 0x64, 0x03, 0xE8, 0x00, 0x64, 0x00, 0x28,
    0x00, 0x00, 0xFF, 0xFD, 0x00, 0x00, 0x00, 0x00, 0x16, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x10, 0x00,
    /* bank # 2 */
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x01, 0x00, 0x05, 0xBA, 0xC6, 0x00, 0x47, 0x78, 0xA2,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x23, 0xBB, 0x00, 0x2E, 0xA2, 0x5B, 0x00, 0x00, 0x05, 0x68, 0x00, 0x0B, 0xCF, 0x49,
    0x00, 0x04, 0xFF, 0xFD, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x00, 0x07, 0x00, 0x08, 0x00, 0x06, 0x00, 0x06, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x2E, 0xA2, 0x5B, 0x00, 0x00, 0x05, 0x68, 0x00, 0x0B, 0xCF, 0x49, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xF8, 0xF6, 0x2A, 0x3F, 0x68, 0xF5, 0x7A, 0x00, 0x04, 0xFF, 0xFD, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x0E,
    0xFF, 0xFF, 0xFF, 0xCF, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xFF, 0xFF, 0xFF, 0x9C,
    0x00, 0x00, 0x43, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
    0xFF, 0xE5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 3 */
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xD3,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3C,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x9E, 0x65, 0x5D,
    0x0C, 0x0A, 0x4E, 0x68, 0xCD, 0xCF, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xC6, 0x19, 0xCE, 0x82,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x71, 0x1C,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xD7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x11, 0xDC, 0x47, 0x03, 0x00, 0x00, 0x00, 0xC7, 0x93, 0x8F, 0x9D, 0x1E, 0x1B, 0x1C, 0x19,
    0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0E, 0xDF, 0xA4, 0x38, 0x1F, 0x9E, 0x65, 0x5D,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x71, 0x1C, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3F, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xF4, 0xC9, 0xFF, 0xFF, 0xBC, 0xF0, 0x00, 0x01, 0x0C, 0x0F,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xF5, 0xB7, 0xBA, 0xB3, 0x67, 0x7D, 0xDF, 0x7E, 0x72, 0x90, 0x2E, 0x55, 0x4C, 0xF6, 0xE6, 0x88,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 4 */
    0xD8, 0xDC, 0xB4, 0xB8, 0xB0, 0xD8, 0xB9, 0xAB, 0xF3, 0xF8, 0xFA, 0xB3, 0xB7, 0xBB, 0x8E, 0x9E,
    0xAE, 0xF1, 0x32, 0xF5, 0x1B, 0xF1, 0xB4, 0xB8, 0xB0, 0x80, 0x97, 0xF1, 0xA9, 0xDF, 0xDF, 0xDF,
    0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0x4C, 0xCD, 0x6C, 0xA9, 0x0C, 0xC9, 0x2C, 0x97, 0xF1, 0xA9,
    0x89, 0x26, 0x46, 0x66, 0xB2, 0x89, 0x99, 0xA9, 0x2D, 0x55, 0x7D, 0xB0, 0xB0, 0x8A, 0xA8, 0x96,
    0x36, 0x56, 0x76, 0xF1, 0xBA, 0xA3, 0xB4, 0xB2, 0x80, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB2, 0x83,
    0x98, 0xBA, 0xA3, 0xF0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xB2, 0xB9, 0xB4, 0x98, 0x83, 0xF1,
    0xA3, 0x29, 0x55, 0x7D, 0xBA, 0xB5, 0xB1, 0xA3, 0x83, 0x93, 0xF0, 0x00, 0x28, 0x50, 0xF5, 0xB2,
    0xB6, 0xAA, 0x83, 0x93, 0x28, 0x54, 0x7C, 0xF1, 0xB9, 0xA3, 0x82, 0x93, 0x61, 0xBA, 0xA2, 0xDA,
    0xDE, 0xDF, 0xDB, 0x81, 0x9A, 0xB9, 0xAE, 0xF5, 0x60, 0x68, 0x70, 0xF1, 0xDA, 0xBA, 0xA2, 0xDF,
    0xD9, 0xBA, 0xA2, 0xFA, 0xB9, 0xA3, 0x82, 0x92, 0xDB, 0x31, 0xBA, 0xA2, 0xD9, 0xBA, 0xA2, 0xF8,
    0xDF, 0x85, 0xA4, 0xD0, 0xC1, 0xBB, 0xAD, 0x83, 0xC2, 0xC5, 0xC7, 0xB8, 0xA2, 0xDF, 0xDF, 0xDF,
    0xBA, 0xA0, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xAA, 0xB3, 0x8D, 0xB4, 0x98, 0x0D, 0x35,
    0x5D, 0xB2, 0xB6, 0xBA, 0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A,
    0xB8, 0xAA, 0x87, 0x2C, 0x54, 0x7C, 0xBA, 0xA4, 0xB0, 0x8A, 0xB6, 0x91, 0x32, 0x56, 0x76, 0xB2,
    0x84, 0x94, 0xA4, 0xC8, 0x08, 0xCD, 0xD8, 0xB8, 0xB4, 0xB0, 0xF1, 0x99, 0x82, 0xA8, 0x2D, 0x55,
    0x7D, 0x98, 0xA8, 0x0E, 0x16, 0x1E, 0xA2, 0x2C, 0x54, 0x7C, 0x92, 0xA4, 0xF0, 0x2C, 0x50, 0x78,
    /* bank # 5 */
    0xF1, 0x84, 0xA8, 0x98, 0xC4, 0xCD, 0xFC, 0xD8, 0x0D, 0xDB, 0xA8, 0xFC, 0x2D, 0xF3, 0xD9, 0xBA,
    0xA6, 0xF8, 0xDA, 0xBA, 0xA6, 0xDE, 0xD8, 0xBA, 0xB2, 0xB6, 0x86, 0x96, 0xA6, 0xD0, 0xF3, 0xC8,
    0x41, 0xDA, 0xA6, 0xC8, 0xF8, 0xD8, 0xB0, 0xB4, 0xB8, 0x82, 0xA8, 0x92, 0xF5, 0x2C, 0x54, 0x88,
    0x98, 0xF1, 0x35, 0xD9, 0xF4, 0x18, 0xD8, 0xF1, 0xA2, 0xD0, 0xF8, 0xF9, 0xA8, 0x84, 0xD9, 0xC7,
    0xDF, 0xF8, 0xF8, 0x83, 0xC5, 0xDA, 0xDF, 0x69, 0xDF, 0x83, 0xC1, 0xD8, 0xF4, 0x01, 0x14, 0xF1,
    0xA8, 0x82, 0x4E, 0xA8, 0x84, 0xF3, 0x11, 0xD1, 0x82, 0xF5, 0xD9, 0x92, 0x28, 0x97, 0x88, 0xF1,
    0x09, 0xF4, 0x1C, 0x1C, 0xD8, 0x84, 0xA8, 0xF3, 0xC0, 0xF9, 0xD1, 0xD9, 0x97, 0x82, 0xF1, 0x29,
    0xF4, 0x0D, 0xD8, 0xF3, 0xF9, 0xF9, 0xD1, 0xD9, 0x82, 0xF4, 0xC2, 0x03, 0xD8, 0xDE, 0xDF, 0x1A,
    0xD8, 0xF1, 0xA2, 0xFA, 0xF9, 0xA8, 0x84, 0x98, 0xD9, 0xC7, 0xDF, 0xF8, 0xF8, 0xF8, 0x83, 0xC7,
    0xDA, 0xDF, 0x69, 0xDF, 0xF8, 0x83, 0xC3, 0xD8, 0xF4, 0x01, 0x14, 0xF1, 0x98, 0xA8, 0x82, 0x2E,
    0xA8, 0x84, 0xF3, 0x11, 0xD1, 0x82, 0xF5, 0xD9, 0x92, 0x50, 0x97, 0x88, 0xF1, 0x09, 0xF4, 0x1C,
    0xD8, 0x84, 0xA8, 0xF3, 0xC0, 0xF8, 0xF9, 0xD1, 0xD9, 0x97, 0x82, 0xF1, 0x49, 0xF4, 0x0D, 0xD8,
    0xF3, 0xF9, 0xF9, 0xD1, 0xD9, 0x82, 0xF4, 0xC4, 0x03, 0xD8, 0xDE, 0xDF, 0xD8, 0xF1, 0xAD, 0x88,
    0x98, 0xCC, 0xA8, 0x09, 0xF9, 0xD9, 0x82, 0x92, 0xA8, 0xF5, 0x7C, 0xF1, 0x88, 0x3A, 0xCF, 0x94,
    0x4A, 0x6E, 0x98, 0xDB, 0x69, 0x31, 0xDA, 0xAD, 0xF2, 0xDE, 0xF9, 0xD8, 0x87, 0x95, 0xA8, 0xF2,
    0x21, 0xD1, 0xDA, 0xA5, 0xF9, 0xF4, 0x17, 0xD9, 0xF1, 0xAE, 0x8E, 0xD0, 0xC0, 0xC3, 0xAE, 0x82,
    /* bank # 6 */
    0xC6, 0x84, 0xC3, 0xA8, 0x85, 0x95, 0xC8, 0xA5, 0x88, 0xF2, 0xC0, 0xF1, 0xF4, 0x01, 0x0E, 0xF1,
    0x8E, 0x9E, 0xA8, 0xC6, 0x3E, 0x56, 0xF5, 0x54, 0xF1, 0x88, 0x72, 0xF4, 0x01, 0x15, 0xF1, 0x98,
    0x45, 0x85, 0x6E, 0xF5, 0x8E, 0x9E, 0x04, 0x88, 0xF1, 0x42, 0x98, 0x5A, 0x8E, 0x9E, 0x06, 0x88,
    0x69, 0xF4, 0x01, 0x1C, 0xF1, 0x98, 0x1E, 0x11, 0x08, 0xD0, 0xF5, 0x04, 0xF1, 0x1E, 0x97, 0x02,
    0x02, 0x98, 0x36, 0x25, 0xDB, 0xF9, 0xD9, 0x85, 0xA5, 0xF3, 0xC1, 0xDA, 0x85, 0xA5, 0xF3, 0xDF,
    0xD8, 0x85, 0x95, 0xA8, 0xF3, 0x09, 0xDA, 0xA5, 0xFA, 0xD8, 0x82, 0x92, 0xA8, 0xF5, 0x78, 0xF1,
    0x88, 0x1A, 0x84, 0x9F, 0x26, 0x88, 0x98, 0x21, 0xDA, 0xF4, 0x1D, 0xF3, 0xD8, 0x87, 0x9F, 0x39,
    0xD1, 0xAF, 0xD9, 0xDF, 0xDF, 0xFB, 0xF9, 0xF4, 0x0C, 0xF3, 0xD8, 0xFA, 0xD0, 0xF8, 0xDA, 0xF9,
    0xF9, 0xD0, 0xDF, 0xD9, 0xF9, 0xD8, 0xF4, 0x0B, 0xD8, 0xF3, 0x87, 0x9F, 0x39, 0xD1, 0xAF, 0xD9,
    0xDF, 0xDF, 0xF4, 0x1D, 0xF3, 0xD8, 0xFA, 0xFC, 0xA8, 0x69, 0xF9, 0xF9, 0xAF, 0xD0, 0xDA, 0xDE,
    0xFA, 0xD9, 0xF8, 0x8F, 0x9F, 0xA8, 0xF1, 0xCC, 0xF3, 0x98, 0xDB, 0x45, 0xD9, 0xAF, 0xDF, 0xD0,
    0xF8, 0xD8, 0xF1, 0x8F, 0x9F, 0xA8, 0xCA, 0xF3, 0x88, 0x09, 0xDA, 0xAF, 0x8F, 0xCB, 0xF8, 0xD8,
    0xF2, 0xAD, 0x97, 0x8D, 0x0C, 0xD9, 0xA5, 0xDF, 0xF9, 0xBA, 0xA6, 0xF3, 0xFA, 0xF4, 0x12, 0xF2,
    0xD8, 0x95, 0x0D, 0xD1, 0xD9, 0xBA, 0xA6, 0xF3, 0xFA, 0xDA, 0xA5, 0xF2, 0xC1, 0xBA, 0xA6, 0xF3,
    0xDF, 0xD8, 0xF1, 0xBA, 0xB2, 0xB6, 0x86, 0x96, 0xA6, 0xD0, 0xCA, 0xF3, 0x49, 0xDA, 0xA6, 0xCB,
    0xF8, 0xD8, 0xB0, 0xB4, 0xB8, 0xD8, 0xAD, 0x84, 0xF2, 0xC0, 0xDF, 0xF1, 0x8F, 0xCB, 0xC3, 0xA8,
    /* bank # 7 */
    0xB2, 0xB6, 0x86, 0x96, 0xC8, 0xC1, 0xCB, 0xC3, 0xF3, 0xB0, 0xB4, 0x88, 0x98, 0xA8, 0x21, 0xDB,
    0x71, 0x8D, 0x9D, 0x71, 0x85, 0x95, 0x21, 0xD9, 0xAD, 0xF2, 0xFA, 0xD8, 0x85, 0x97, 0xA8, 0x28,
    0xD9, 0xF4, 0x08, 0xD8, 0xF2, 0x8D, 0x29, 0xDA, 0xF4, 0x05, 0xD9, 0xF2, 0x85, 0xA4, 0xC2, 0xF2,
    0xD8, 0xA8, 0x8D, 0x94, 0x01, 0xD1, 0xD9, 0xF4, 0x11, 0xF2, 0xD8, 0x87, 0x21, 0xD8, 0xF4, 0x0A,
    0xD8, 0xF2, 0x84, 0x98, 0xA8, 0xC8, 0x01, 0xD1, 0xD9, 0xF4, 0x11, 0xD8, 0xF3, 0xA4, 0xC8, 0xBB,
    0xAF, 0xD0, 0xF2, 0xDE, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xD8, 0xF1, 0xB8, 0xF6,
    0xB5, 0xB9, 0xB0, 0x8A, 0x95, 0xA3, 0xDE, 0x3C, 0xA3, 0xD9, 0xF8, 0xD8, 0x5C, 0xA3, 0xD9, 0xF8,
    0xD8, 0x7C, 0xA3, 0xD9, 0xF8, 0xD8, 0xF8, 0xF9, 0xD1, 0xA5, 0xD9, 0xDF, 0xDA, 0xFA, 0xD8, 0xB1,
    0x85, 0x30, 0xF7, 0xD9, 0xDE, 0xD8, 0xF8, 0x30, 0xAD, 0xDA, 0xDE, 0xD8, 0xF2, 0xB4, 0x8C, 0x99,
    0xA3, 0x2D, 0x55, 0x7D, 0xA0, 0x83, 0xDF, 0xDF, 0xDF, 0xB5, 0x91, 0xA0, 0xF6, 0x29, 0xD9, 0xFB,
    0xD8, 0xA0, 0xFC, 0x29, 0xD9, 0xFA, 0xD8, 0xA0, 0xD0, 0x51, 0xD9, 0xF8, 0xD8, 0xFC, 0x51, 0xD9,
    0xF9, 0xD8, 0x79, 0xD9, 0xFB, 0xD8, 0xA0, 0xD0, 0xFC, 0x79, 0xD9, 0xFA, 0xD8, 0xA1, 0xF9, 0xF9,
    0xF9, 0xF9, 0xF9, 0xA0, 0xDA, 0xDF, 0xDF, 0xDF, 0xD8, 0xA1, 0xF8, 0xF8, 0xF8, 0xF8, 0xF8, 0xAC,
    0xDE, 0xF8, 0xAD, 0xDE, 0x83, 0x93, 0xAC, 0x2C, 0x54, 0x7C, 0xF1, 0xA8, 0xDF, 0xDF, 0xDF, 0xF6,
    0x9D, 0x2C, 0xDA, 0xA0, 0xDF, 0xD9, 0xFA, 0xDB, 0x2D, 0xF8, 0xD8, 0xA8, 0x50, 0xDA, 0xA0, 0xD0,
    0xDE, 0xD9, 0xD0, 0xF8, 0xF8, 0xF8, 0xDB, 0x55, 0xF8, 0xD8, 0xA8, 0x78, 0xDA, 0xA0, 0xD0, 0xDF,
    /* bank # 8 */
    0xD9, 0xD0, 0xFA, 0xF8, 0xF8, 0xF8, 0xF8, 0xDB, 0x7D, 0xF8, 0xD8, 0x9C, 0xA8, 0x8C, 0xF5, 0x30,
    0xDB, 0x38, 0xD9, 0xD0, 0xDE, 0xDF, 0xA0, 0xD0, 0xDE, 0xDF, 0xD8, 0xA8, 0x48, 0xDB, 0x58, 0xD9,
    0xDF, 0xD0, 0xDE, 0xA0, 0xDF, 0xD0, 0xDE, 0xD8, 0xA8, 0x68, 0xDB, 0x70, 0xD9, 0xDF, 0xDF, 0xA0,
    0xDF, 0xDF, 0xD8, 0xF1, 0xA8, 0x88, 0x90, 0x2C, 0x54, 0x7C, 0x98, 0xA8, 0xD0, 0x5C, 0x38, 0xD1,
    0xDA, 0xF2, 0xAE, 0x8C, 0xDF, 0xF9, 0xD8, 0xB0, 0x87, 0xA8, 0xC1, 0xC1, 0xB1, 0x88, 0xA8, 0xC6,
    0xF9, 0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xA8,
    0xF9, 0xDA, 0x36, 0xD8, 0xA8, 0xF9, 0xDA, 0x36, 0xD8, 0xF7, 0x8D, 0x9D, 0xAD, 0xF8, 0x18, 0xDA,
    0xF2, 0xAE, 0xDF, 0xD8, 0xF7, 0xAD, 0xFA, 0x30, 0xD9, 0xA4, 0xDE, 0xF9, 0xD8, 0xF2, 0xAE, 0xDE,
    0xFA, 0xF9, 0x83, 0xA7, 0xD9, 0xC3, 0xC5, 0xC7, 0xF1, 0x88, 0x9B, 0xA7, 0x7A, 0xAD, 0xF7, 0xDE,
    0xDF, 0xA4, 0xF8, 0x84, 0x94, 0x08, 0xA7, 0x97, 0xF3, 0x00, 0xAE, 0xF2, 0x98, 0x19, 0xA4, 0x88,
    0xC6, 0xA3, 0x94, 0x88, 0xF6, 0x32, 0xDF, 0xF2, 0x83, 0x93, 0xDB, 0x09, 0xD9, 0xF2, 0xAA, 0xDF,
    0xD8, 0xD8, 0xAE, 0xF8, 0xF9, 0xD1, 0xDA, 0xF3, 0xA4, 0xDE, 0xA7, 0xF1, 0x88, 0x9B, 0x7A, 0xD8,
    0xF3, 0x84, 0x94, 0xAE, 0x19, 0xF9, 0xDA, 0xAA, 0xF1, 0xDF, 0xD8, 0xA8, 0x81, 0xC0, 0xC3, 0xC5,
    0xC7, 0xA3, 0x92, 0x83, 0xF6, 0x28, 0xAD, 0xDE, 0xD9, 0xF8, 0xD8, 0xA3, 0x50, 0xAD, 0xD9, 0xF8,
    0xD8, 0xA3, 0x78, 0xAD, 0xD9, 0xF8, 0xD8, 0xF8, 0xF9, 0xD1, 0xA1, 0xDA, 0xDE, 0xC3, 0xC5, 0xC7,
    0xD8, 0xA1, 0x81, 0x94, 0xF8, 0x18, 0xF2, 0xB0, 0x89, 0xAC, 0xC3, 0xC5, 0xC7, 0xF1, 0xD8, 0xB8,
    /* bank # 9 */
    0xB4, 0xB0, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97, 0x28, 0x88, 0x9B, 0xF0,
    0x0C, 0x20, 0x14, 0x40, 0xB0, 0xB4, 0xB8, 0xF0, 0xA8, 0x8A, 0x9A, 0x28, 0x50, 0x78, 0xB7, 0x9B,
    0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xF1, 0xBB, 0xAB,
    0x88, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0xB3, 0x8B, 0xB8, 0xA8, 0x04, 0x28, 0x50, 0x78, 0xF1, 0xB0,
    0x88, 0xB4, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xBB, 0xAB, 0xB3, 0x8B, 0x02, 0x26, 0x46, 0x66, 0xB0,
    0xB8, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79, 0x8A, 0x24, 0x70, 0x59,
    0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68, 0x8A, 0x64, 0x48, 0x31,
    0x8B, 0x30, 0x49, 0x60, 0x88, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04, 0x28,
    0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66, 0xF0,
    0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xA9,
    0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60, 0x8C,
    0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76, 0x7E,
    0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0xD8, 0xB1, 0xB5, 0xB9, 0xA3, 0xDF, 0xDF, 0xDF, 0xAE, 0xD0,
    0xDF, 0xAA, 0xD0, 0xDE, 0xF2, 0xAB, 0xF8, 0xF9, 0xD9, 0xB0, 0x87, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF,
    0xBB, 0xAF, 0xDF, 0xDF, 0xB9, 0xD8, 0xB1, 0xF1, 0xA3, 0x97, 0x8E, 0x60, 0xDF, 0xB0, 0x84, 0xF2,
    0xC8, 0xF8, 0xF9, 0xD9, 0xDE, 0xD8, 0x93, 0x85, 0xF1, 0x4A, 0xB1, 0x83, 0xA3, 0x08, 0xB5, 0x83,
    /* bank # 10 */
    0x9A, 0x08, 0x10, 0xB7, 0x9F, 0x10, 0xD8, 0xF1, 0xB0, 0xBA, 0xAE, 0xB0, 0x8A, 0xC2, 0xB2, 0xB6,
    0x8E, 0x9E, 0xF1, 0xFB, 0xD9, 0xF4, 0x1D, 0xD8, 0xF9, 0xD9, 0x0C, 0xF1, 0xD8, 0xF8, 0xF8, 0xAD,
    0x61, 0xD9, 0xAE, 0xFB, 0xD8, 0xF4, 0x0C, 0xF1, 0xD8, 0xF8, 0xF8, 0xAD, 0x19, 0xD9, 0xAE, 0xFB,
    0xDF, 0xD8, 0xF4, 0x16, 0xF1, 0xD8, 0xF8, 0xAD, 0x8D, 0x61, 0xD9, 0xF4, 0xF4, 0xAC, 0xF5, 0x9C,
    0x9C, 0x8D, 0xDF, 0x2B, 0xBA, 0xB6, 0xAE, 0xFA, 0xF8, 0xF4, 0x0B, 0xD8, 0xF1, 0xAE, 0xD0, 0xF8,
    0xAD, 0x51, 0xDA, 0xAE, 0xFA, 0xF8, 0xF1, 0xD8, 0xB9, 0xB1, 0xB6, 0xA3, 0x83, 0x9C, 0x08, 0xB9,
    0xB1, 0x83, 0x9A, 0xB5, 0xAA, 0xC0, 0xFD, 0x30, 0x83, 0xB7, 0x9F, 0x10, 0xB5, 0x8B, 0x93, 0xF2,
    0x02, 0x02, 0xD1, 0xAB, 0xDA, 0xDE, 0xD8, 0xF1, 0xB0, 0x80, 0xBA, 0xAB, 0xC0, 0xC3, 0xB2, 0x84,
    0xC1, 0xC3, 0xD8, 0xB1, 0xB9, 0xF3, 0x8B, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xB0,
    0x87, 0x9C, 0xB9, 0xA3, 0xDD, 0xF1, 0xB3, 0x8B, 0x8B, 0x8B, 0x8B, 0x8B, 0xB0, 0x87, 0x20, 0x28,
    0x30, 0x38, 0xB2, 0x8B, 0xB6, 0x9B, 0xF2, 0xA3, 0xC0, 0xC8, 0xC2, 0xC4, 0xCC, 0xC6, 0xA3, 0xA3,
    0xA3, 0xF1, 0xB0, 0x87, 0xB5, 0x9A, 0xD8, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC, 0xBA, 0xAC, 0xDF, 0xB9, //Reverted back as packet size changes causing isues... TODO:change 2742 from 0xD8 to 0x20 Including the DMP_FEATURE_TAP -- known issue in which if you do not enable DMP_FEATURE_TAP then the interrupts will be at 200Hz even if fifo rate
    0xA3, 0xFE, 0xF2, 0xAB, 0xC4, 0xAA, 0xF1, 0xDF, 0xDF, 0xBB, 0xAF, 0xDF, 0xDF, 0xA3, 0xA3, 0xA3,
    0xD8, 0xD8, 0xD8, 0xBB, 0xB3, 0xB7, 0xF1, 0xAA, 0xF9, 0xDA, 0xFF, 0xD9, 0x80, 0x9A, 0xAA, 0x28,
    0xB4, 0x80, 0x98, 0xA7, 0x20, 0xB7, 0x97, 0x87, 0xA8, 0x66, 0x88, 0xF0, 0x79, 0x51, 0xF1, 0x90,
    0x2C, 0x87, 0x0C, 0xA7, 0x81, 0x97, 0x62, 0x93, 0xF0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
    /* bank # 11 */
    0x51, 0x79, 0x90, 0xA5, 0xF1, 0x28, 0x4C, 0x6C, 0x87, 0x0C, 0x95, 0x18, 0x85, 0x78, 0xA3, 0x83,
    0x90, 0x28, 0x4C, 0x6C, 0x88, 0x6C, 0xD8, 0xF3, 0xA2, 0x82, 0x00, 0xF2, 0x10, 0xA8, 0x92, 0x19,
    0x80, 0xA2, 0xF2, 0xD9, 0x26, 0xD8, 0xF1, 0x88, 0xA8, 0x4D, 0xD9, 0x48, 0xD8, 0x96, 0xA8, 0x39,
    0x80, 0xD9, 0x3C, 0xD8, 0x95, 0x80, 0xA8, 0x39, 0xA6, 0x86, 0x98, 0xD9, 0x2C, 0xDA, 0x87, 0xA7,
    0x2C, 0xD8, 0xA8, 0x89, 0x95, 0x19, 0xA9, 0x80, 0xD9, 0x38, 0xD8, 0xA8, 0x89, 0x39, 0xA9, 0x80,
    0xDA, 0x3C, 0xD8, 0xA8, 0x2E, 0xA8, 0x39, 0x90, 0xD9, 0x0C, 0xD8, 0xA8, 0x95, 0x31, 0x98, 0xD9,
    0x0C, 0xD8, 0xA8, 0x09, 0xD9, 0xFF, 0xD8, 0x01, 0xDA, 0xFF, 0xD8, 0x95, 0x39, 0xA9, 0xDA, 0x26,
    0xFF, 0xD8, 0x90, 0xA8, 0x0D, 0x89, 0x99, 0xA8, 0x10, 0x80, 0x98, 0x21, 0xDA, 0x2E, 0xD8, 0x89,
    0x99, 0xA8, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8, 0x86, 0x96, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8,
    0x87, 0x31, 0x80, 0xDA, 0x2E, 0xD8, 0xA8, 0x82, 0x92, 0xF3, 0x41, 0x80, 0xF1, 0xD9, 0x2E, 0xD8,
    0xA8, 0x82, 0xF3, 0x19, 0x80, 0xF1, 0xD9, 0x2E, 0xD8, 0x82, 0xAC, 0xF3, 0xC0, 0xA2, 0x80, 0x22,
    0xF1, 0xA6, 0x2E, 0xA7, 0x2E, 0xA9, 0x22, 0x98, 0xA8, 0x29, 0xDA, 0xAC, 0xDE, 0xFF, 0xD8, 0xA2,
    0xF2, 0x2A, 0xF1, 0xA9, 0x2E, 0x82, 0x92, 0xA8, 0xF2, 0x31, 0x80, 0xA6, 0x96, 0xF1, 0xD9, 0x00,
    0xAC, 0x8C, 0x9C, 0x0C, 0x30, 0xAC, 0xDE, 0xD0, 0xDE, 0xFF, 0xD8, 0x8C, 0x9C, 0xAC, 0xD0, 0x10,
    0xAC, 0xDE, 0x80, 0x92, 0xA2, 0xF2, 0x4C, 0x82, 0xA8, 0xF1, 0xCA, 0xF2, 0x35, 0xF1, 0x96, 0x88,
    0xA6, 0xD9, 0x00, 0xD8, 0xF1, 0xFF]


  static REG-BANK-SEL_               ::= 0x6d
  static REG-BANK-SEL-MEM-SEL-MASK_  ::= 0b00011111
  static REG-MEM-START-ADDR_         ::= 0x6e
  static REG-MEM-R-W_                ::= 0x6f
  static DMP-MEM-CHUNK-SIZE_         ::= 16   // safe default (can increase if you like)

  // DMP Feature Constants
  static DMP-FEATURE-TAP_            ::= 0x001
  static DMP-FEATURE-ANDROID-ORIENT_ ::= 0x002
  static DMP-FEATURE-LP-QUAT_        ::= 0x004
  static DMP-FEATURE-PEDOMETER_      ::= 0x008
  static DMP-FEATURE-6X-LP-QUAT_     ::= 0x010
  static DMP-FEATURE-GYRO-CAL_       ::= 0x020
  static DMP-FEATURE-SEND-RAW-ACCEL_ ::= 0x040
  static DMP-FEATURE-SEND-RAW-GYRO_  ::= 0x080
  static DMP-FEATURE-SEND-CAL-GYRO_  ::= 0x100

  //static USE_CAL_HW_REGISTERS

  // DMP-FIFO-RATE-DIVISOR is pre configured into the above image and can't be modified at this time.
  static DMP-FIFO-RATE-DIVISOR ::= 0x01
  static REG-DMP-CFG_ ::= 0x70 // 16 bit

  // DMP Registers
  static REG-DMP-INT-STATUS  ::= 0x39
  static DMP-INT-STATUS-INT-5-MASK_ ::= 0b00100000
  static DMP-INT-STATUS-INT-4-MASK_ ::= 0b00010000
  static DMP-INT-STATUS-INT-3-MASK_ ::= 0b00001000
  static DMP-INT-STATUS-INT-2-MASK_ ::= 0b00000100
  static DMP-INT-STATUS-INT-1-MASK_ ::= 0b00000010
  static DMP-INT-STATUS-INT-0-MASK_ ::= 0b00000001

  // Valid for REG-INTRPT-ENABLE_ (0x38) and REG-INTRPT-STATUS_ (0x3a)
  static INTRPT-DMP-MASK_ ::= 0b00000010

  // Valid for REG-USER-CTRL (0x6a)
  static USER-DMP-ENABLE-MASK_ ::= 0b10000000
  static USER-DMP-RESET-MASK_  ::= 0b00001000  // Requires USER-DMP-ENABLE_ = 0

  // DMP packet size (e.g. 27 for MotionApps 6.12 quaternion packet)
  static DEFAULT-DMP-PACKET-SIZE_  ::= 28
  /** ======================================================================= *
  | Default MotionApps v6.12 28-byte FIFO packet structure:                  |
  |                                                                          |
  | [QUAT W ][       ][QUAT X ][       ][QUAT Y ][       ][QUAT Z ][       ] |
  |   0  1     2   3    4   5    6   7    8   9    10 11    12 13    14 15   |
  |                                                                          |
  | [GYRO X ][GYRO Y ][GYRO Z ][ACC  X ][ACC  Y ][ACC  Z ]                   |
  |   16 17    18 19    20 21    22 23    24 25    26 27                     |
  * ======================================================================= */

  // Globals
  reg_/registers.Registers := ?
  logger_/log.Logger := ?
  dmp-firmware-uploaded_/bool := ?
  dmp-capable_ := true
  dmp-packet-size_ := DEFAULT-DMP-PACKET-SIZE_

  accel-lsb-per-g_/float  := 16384.0   // AFS_SEL = ±2 g
  gyro-lsb-per-dps_/float := 16.4    // FS_SEL = ±2000 °/s

  /** Class Constructor:  */
  constructor
      device/serial.Device
      --logger/log.Logger=log.default:
    logger_ = logger.with-name "mpu6050-dmp-ma612"
    reg_ = device.registers
    dmp-firmware-uploaded_ = false
    super device

  /**
  Enables Digital Motion Processor Capability.
  */
  enable-dmp -> none:
    // Ported version of jrowberg's initialise implementation, which is itself
    // implemented from the "MPU-6000/MPU-6050 Register Map and Descriptions"
    // page 41

    // Can't do this yet - bus doesn't tolerate the reset
    //I2Cdev::writeBit(devAddr,0x6B, 7, (val = 1), wireObj); //PWR_MGMT_1: reset with 100ms delay
    //delay(100)

    // Full SIGNAL_PATH_RESET: with another 100ms delay
    //I2Cdev::writeBits(devAddr,0x6A, 2, 3, (val = 0b111), wireObj);
    write-register_ Mpu6050.REG-USER-CTRL_ 0b111 --width=8
    sleep --ms=100

    // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
    //write-register_ Mpu6050.REG-POWER-MANAGEMENT_ 0x1 --mask=Mpu6050.PM-LP-WAKE-CTRL-MASK_ --width=16
    wakeup-now
    set-clock-source CLOCK-SRC-PLL-X-G
    sleep --ms=10

    // INT_ENABLE: no Interrupts
    disable-all-interrupts

    // MPU FIFO_EN: (all off) Using DMP's FIFO instead
    disable-fifo

    // ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
    set-accelerometer-high-pass-filter 0

    // 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
    set-interrupt-pin-active-low

    // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
    set-clock-source 0x01

    // 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    write-register_ 0x19 0x04 --width=8

    // 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ
    set-dlpf-config Mpu6050.CONFIG-DLPF-1

    //if (!writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) return 1; // Loads the DMP image into the MPU6050 Memory // Should Never Fail
    if not dmp-firmware-uploaded_:
      dmp-firmware-uploaded_ = dmp-write-bytes_ dmp-ma-612_ --address=0x00 --verify

    // DMP Program Start Address
    // I2Cdev::writeWords(devAddr, 0x70, 1, &(ival = 0x0400), wireObj);
    write-register_ REG-DMP-CFG_ 0x0400 --width=16









    // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
    set-gyroscope-full-scale Mpu6050.GYRO-FS-16-4

    //I2Cdev::writeBytes(devAddr,0x6A, 1, &(val = 0xC0), wireObj);
    // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
    enable-fifo

    // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
    enable-interrupt-dmp

    set-sample-rate-hz 200

    //I2Cdev::writeBit(devAddr,0x6A, 2, 1, wireObj);
    // Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 bit and then saves the byte)
    reset-fifo

    // Cache DMP Constants
    set-dmp-constants_

    // Write the enable register
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=USER-DMP-ENABLE-MASK_ --width=8

    // Reset removes gesture footer to the default packet size.
    dmp-packet-size_ = DEFAULT-DMP-PACKET-SIZE_



  // Resets FIFO safely (you can also pulse DMP reset if you want a harder recover)
  clear-fifo -> none:
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=Mpu6050.USER-FIFO-RESET-MASK_ --width=8

  buffer-size -> int:
    return read-register_ Mpu6050.REG-FIFO-COUNT_ --width=16


  /**
  Read one packet from the buffer.

  Automatically recovers from overflow and desync.
  */
  read-dmp-packet --drain=false -> ByteArray?:
    // Check for FIFO overflow and recover.
    interrupt-status := read-register_ Mpu6050.REG-INTRPT-STATUS_ --width=8
    if (interrupt-status & Mpu6050.INTRPT-FIFO-OVERFLOW-MASK_) != 0:
      logger_.error "read-dmp-packet: fifo overflow - resetting." --tags={"status":bits-16_ interrupt-status}
      clear-fifo-after-dmp-overflow
      return null

    byte-count := read-register_ Mpu6050.REG-FIFO-COUNT_ --width=16
    if byte-count < dmp-packet-size_:
      logger_.error "read-dmp-packet: not enough bytes in buffer" --tags={"bytes":byte-count}
      return null

    // If not an exact multiple, it's desynced.  Easiest safe recovery: reset FIFO.
    remainder := byte-count % dmp-packet-size_
    if remainder != 0:
      write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=Mpu6050.USER-FIFO-RESET-MASK_ --width=8
      logger_.error "read-dmp-packet: desynced." --tags={"remainder":remainder}
      return null

    if not drain:
      // Read exactly one packet (you can loop to read multiple if you prefer).
      pkt := reg_.read-bytes Mpu6050.REG-FIFO-RW_ dmp-packet-size_
      return pkt
    else:
      // Read everything that's there (could be many packets)
      packet-count := byte-count / dmp-packet-size_
      total-bytes := packet-count * dmp-packet-size_
      packet-blob := reg_.read-bytes Mpu6050.REG-FIFO-RW_ total-bytes

      // Return the last packet (newest), older ones implicitly dropped
      start := total-bytes - dmp-packet-size_
      return packet-blob[start .. start + dmp-packet-size_]

  clear-fifo-after-dmp-overflow -> none:
    // stop DMP writing while we clear things
    write-register_ Mpu6050.REG-USER-CTRL_ 0 --mask=USER-DMP-ENABLE-MASK_ --width=8

    // reset FIFO and DMP core
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=Mpu6050.USER-FIFO-RESET-MASK_ --width=8
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=USER-DMP-RESET-MASK_        --width=8

    // clear latched INTs
    nothing := read-register_ Mpu6050.REG-INTRPT-STATUS_ --width=8

    // re-enable FIFO and DMP
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=Mpu6050.USER-FIFO-ENABLE-MASK_ --width=8
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=USER-DMP-ENABLE-MASK_          --width=8

  set-dmp-constants_ -> none:
    gyro-fs := get-gyroscope-full-scale          // Obtain range configuration
    gyro-lsb := convert-gyro-fs-to-value_ gyro-fs     // Obtain multiplier
    gyro-lsb-per-dps_ = gyro-lsb
    accel-fs := get-accelerometer-full-scale          // Obtain range configuration
    accel-lsb := convert-accel-fs-to-value_ accel-fs     // Obtain multiplier
    accel-lsb-per-g_ = accel-lsb
    logger_.info "set-dmp-constants_: " --tags={"gyro-lsb-per-dps_":gyro-lsb-per-dps_,"accel-lsb-per-g_":accel-lsb-per-g_}


  read-accelerometer-dmp --set/Point3i=(read-accelerometer-dmp-raw) -> Point3f:
    accel-x := set.x.to-float / accel-lsb-per-g_
    accel-y := set.y.to-float / accel-lsb-per-g_
    accel-z := set.z.to-float / accel-lsb-per-g_
    return Point3f accel-x accel-y accel-z

  /**
  Reads accelerometer at this moment, returning dmp fifo reads.
  */
  read-accelerometer-dmp-raw packet/ByteArray?=null -> Point3i:
    if packet == null: packet = read-dmp-packet --drain
    if packet == null: return Point3i 0 0 0  // avoid crash
    accel-x := i16-be_ packet[22] packet[23]
    accel-y := i16-be_ packet[24] packet[25]
    accel-z := i16-be_ packet[26] packet[27]
    return Point3i accel-x accel-y accel-z


  read-gyroscope-dmp --set/Point3i=(read-gyroscope-dmp-raw) -> Point3f:
    gyro-x := set.x.to-float / gyro-lsb-per-dps_
    gyro-y := set.y.to-float / gyro-lsb-per-dps_
    gyro-z := set.z.to-float / gyro-lsb-per-dps_
    return Point3f gyro-x gyro-y gyro-z

  /**
  Reads gyroscope at this moment, returning dmp fifo reads.
  */
  read-gyroscope-dmp-raw packet/ByteArray?=null -> Point3i:
    if packet == null: packet = read-dmp-packet --drain
    if packet == null: return Point3i 0 0 0  // avoid crash
    gyro-x := i16-be_ packet[16] packet[17]
    gyro-y := i16-be_ packet[18] packet[19]
    gyro-z := i16-be_ packet[20] packet[21]
    return Point3i gyro-x gyro-y gyro-z


/*
  read-linear-acceleration --set/Point3i=(read-accelerometer-dmp-raw) gravity/float -> Point3f:
    // get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x*16384
    v -> y = vRaw -> y - gravity -> y*16384
    v -> z = vRaw -> z - gravity -> z*16384
    return Point3f
*/



/** ======================================================================= *
 | Default MotionApps v6.12 28-byte FIFO packet structure:                  |
 |                                                                          |
 | [QUAT W ][       ][QUAT X ][       ][QUAT Y ][       ][QUAT Z ][       ] |
 |   0  1     2   3    4   5    6   7    8   9    10 11    12 13    14 15   |
 |                                                                          |
 | [GYRO X ][GYRO Y ][GYRO Z ][ACC  X ][ACC  Y ][ACC  Z ]                   |
 |   16 17    18 19    20 21    22 23    24 25    26 27                     |
 * ======================================================================= */


  read-quaternion-dmp --set/Vector4i=(read-quaternion-dmp-raw) -> Vector4f:
    q30-s := 1073741824.0  // 2^30
    quat-w := set.w.to-float / q30-s
    quat-x := set.x.to-float / q30-s
    quat-y := set.y.to-float / q30-s
    quat-z := set.z.to-float / q30-s
    return Vector4f quat-w quat-x quat-y quat-z

  /**
  Reads gyroscope at this moment, returning dmp fifo reads.
  */
  read-quaternion-dmp-raw packet/ByteArray?=null -> Vector4i:
    if packet == null: packet = read-dmp-packet --drain
    if packet == null: return Vector4i 0 0 0 0  // avoid crash
    quat-w := i32-be_ packet --start=0 --signed
    quat-x := i32-be_ packet --start=4 --signed
    quat-y := i32-be_ packet --start=8 --signed
    quat-z := i32-be_ packet --start=12 --signed
    return Vector4i quat-w quat-x quat-y quat-z


  /**
  Disables Digital Motion Processor Capability.
  */
  disable-dmp -> none:
    write-register_ Mpu6050.REG-USER-CTRL_ 0 --mask=USER-DMP-ENABLE-MASK_ --width=8


  /**
  Resets DMP capability.

  reset-dmp should assert USER_CTRL.DMP_RESET then re-load the image if needed.

  Must be off to reset - reads previous state, disables and restores the
   previous state after reset.
  */
  reset-dmp -> none:
    enabled := read-register_ Mpu6050.REG-USER-CTRL_ --mask=USER-DMP-ENABLE-MASK_ --width=8
    write-register_ Mpu6050.REG-USER-CTRL_ 0 --mask=USER-DMP-ENABLE-MASK_ --width=8
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=USER-DMP-RESET-MASK_ --width=8
    sleep --ms=100
    if enabled == 1:
      enable-dmp

  /**
  In raw mode you choose which sensors feed FIFO (REG-FIFO-EN bits).
  In DMP mode the DMP writes its own packets into FIFO; you normally clear all raw sensor bits in REG-FIFO-EN and let DMP do the writing.
  */
  enable-fifo -> none:
    // Do the basic Enable Fifo
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=Mpu6050.USER-FIFO-RESET-MASK_ --width=8
    write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=Mpu6050.USER-FIFO-ENABLE-MASK_ --width=8
    if is-dmp-enabled:
      write-register_ Mpu6050.REG-USER-CTRL_ 1 --mask=USER-DMP-RESET-MASK_  --width=8

  disable-fifo -> none:
    if is-dmp-enabled:
      logger_.error "disable-all-fifo: Cannot use in DMP mode."
    write-register_ Mpu6050.REG-USER-CTRL_ 0 --mask=Mpu6050.USER-FIFO-ENABLE-MASK_ --width=8
    write-register_ Mpu6050.REG-FIFO-EN_ 0 --width=8

  enable-interrupt-dmp -> none:
    write-register_ Mpu6050.REG-INTRPT-ENABLE_ 1 --mask=INTRPT-DMP-MASK_ --width=8

  disable-interrupt-dmp -> none:
    write-register_ Mpu6050.REG-INTRPT-ENABLE_ 0 --mask=INTRPT-DMP-MASK_  --width=8

  // Set current DMP memory bank [typically 0..7]
  set-mem-write-bank_ bank/int -> none:
    write-register_ REG-BANK-SEL_ bank --mask=REG-BANK-SEL-MEM-SEL-MASK_ --width=8

  // Set start address within current bank (0..255)
  set-mem-write-offset_ offset/int -> none:
    write-register_ REG-MEM-START-ADDR_ offset --width=8

  /**
  Write a DMP memory block, handling 256-byte bank boundaries.
  - data: ByteArray (bytes) to write
  - bank: starting bank
  - address: starting address within bank (0..255)
  - --verify: if true, read back and compare
  Returns true on success (or false if verify fails).
  */
  dmp-write-bytes_ data/ByteArray --address/int --verify/bool=false -> bool:
    bank := dmp-bank_ address
    offset := dmp-offset_ address
    if data.size == 0:
      logger_.error "write-dmp-memory-block: given 0 size data."
      return true

    // scratch buffer once for verify reads (max = chunk size)
    verify-buf/ByteArray? := verify ? ByteArray DMP-MEM-CHUNK-SIZE_ : null

    i := 0
    while i < data.size:
      // Advise where data is going
      set-mem-write-bank_ bank
      set-mem-write-offset_ offset
      logger_.info "write-memory-block: Writing...     " --tags={"bank":bank,"offset":offset}

      // choose chunk bounded by: our max chunk, remaining data, bytes left in this bank
      chunk := DMP-MEM-CHUNK-SIZE_
      remaining := data.size - i
      if chunk > remaining: chunk = remaining
      room := 256 - offset
      if chunk > room: chunk = room

      // slice for this burst (zero-copy view on ByteArray)
      slice := data[i .. i + chunk]

      // burst write to MEM_R_W (auto-increments internal address)
      reg_.write-bytes REG-MEM-R-W_ slice

      if verify:
        // Restart from bank/address and burst read back to compare
        set-mem-write-bank_ bank
        set-mem-write-offset_ offset


        // read exactly 'chunk' bytes
        verify-bytes := reg_.read-bytes REG-MEM-R-W_ chunk

        // compare (ByteArray supports index; do manual compare)
        j := 0
        ok := true
        while j < chunk:
          if (slice[j] & 0xFF) != (verify-bytes[j] & 0xFF):

            ok = false
            break
          j += 1
        if ok:
          logger_.info "write-memory-block: Verifying... OK" --tags={"bank":bank,"offset":offset}
        else:
          logger_.error "write-dmp-memory-block: verify failed." --tags={"bank":bank,"offset":offset,"byte" : ( j + i ) }
          return false

      // advance within bank
      i += chunk
      offset = (offset + chunk) & 0xFF

      // if more to write, move to next bank on wrap and set start
      if i < data.size:
        if offset == 0: bank += 1

    return true

  // Read N bytes from a linear address, crossing banks safely.
  dmp-read-bytes_ --address/int --length/int -> ByteArray:
    output := ByteArray length
    bank := dmp-bank_ address
    offset := dmp-offset_ address

    // Open Output Array
    bytes/ByteArray := #[]

    i := 0
    while i < length:
      set-mem-write-bank_ bank
      set-mem-write-offset_ offset

      room := 256 - offset
      chunk := DMP-MEM-CHUNK-SIZE_
      rem := length - i
      if chunk > rem:  chunk = rem
      if chunk > room: chunk = room

      bytes = reg_.read-bytes REG-MEM-R-W_ chunk

      // copy to output variable
      k := 0
      while k < chunk:
        output[i + k] = bytes[k]
        k += 1
      i += chunk
      offset = (offset + chunk) & 0xFF
      if offset == 0 and i < length: bank += 1

    return output

/*


// this is the most basic initialization I can create. with the intent that we access the register bytes as few times as needed to get the job done.
// for detailed descriptins of all registers and there purpose google "MPU-6000/MPU-6050 Register Map and Descriptions"
uint8_t MPU6050::dmpInitialize() { // Lets get it over with fast Write everything once and set it up necely
	uint8_t val;
	uint16_t ival;
  // Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and Descriptions" page 41
	I2Cdev::writeBit(devAddr,0x6B, 7, (val = 1), wireObj); //PWR_MGMT_1: reset with 100ms delay
	delay(100);
	I2Cdev::writeBits(devAddr,0x6A, 2, 3, (val = 0b111), wireObj); // full SIGNAL_PATH_RESET: with another 100ms delay
	delay(100);
	I2Cdev::writeBytes(devAddr,0x6B, 1, &(val = 0x01), wireObj); // 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
	I2Cdev::writeBytes(devAddr,0x38, 1, &(val = 0x00), wireObj); // 0000 0000 INT_ENABLE: no Interrupt
	I2Cdev::writeBytes(devAddr,0x23, 1, &(val = 0x00), wireObj); // 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
	I2Cdev::writeBytes(devAddr,0x1C, 1, &(val = 0x00), wireObj); // 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
	I2Cdev::writeBytes(devAddr,0x37, 1, &(val = 0x80), wireObj); // 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
	I2Cdev::writeBytes(devAddr,0x6B, 1, &(val = 0x01), wireObj); // 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
	I2Cdev::writeBytes(devAddr,0x19, 1, &(val = 0x04), wireObj); // 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
	I2Cdev::writeBytes(devAddr,0x1A, 1, &(val = 0x01), wireObj); // 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
	if (!writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) return 1; // Loads the DMP image into the MPU6050 Memory // Should Never Fail
	I2Cdev::writeWords(devAddr, 0x70, 1, &(ival = 0x0400), wireObj); // DMP Program Start Address
	I2Cdev::writeBytes(devAddr,0x1B, 1, &(val = 0x18), wireObj); // 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
	I2Cdev::writeBytes(devAddr,0x6A, 1, &(val = 0xC0), wireObj); // 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
	I2Cdev::writeBytes(devAddr,0x38, 1, &(val = 0x02), wireObj); // 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
	I2Cdev::writeBit(devAddr,0x6A, 2, 1, wireObj);      // Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 bit and then saves the byte)

  setDMPEnabled(false); // disable DMP for compatibility with the MPU6050 library
/*
    dmpPacketSize += 16;//DMP_FEATURE_6X_LP_QUAT
    dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_ACCEL
    dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_GYRO
*/
	dmpPacketSize = 28;
	return 0;
}

bool MPU6050::dmpPacketAvailable() {
    return getFIFOCount() >= dmpGetFIFOPacketSize();
}

// uint8_t MPU6050::dmpSetFIFORate(uint8_t fifoRate);
// uint8_t MPU6050::dmpGetFIFORate();
// uint8_t MPU6050::dmpGetSampleStepSizeMS();
// uint8_t MPU6050::dmpGetSampleFrequency();
// int32_t MPU6050::dmpDecodeTemperature(int8_t tempReg);

//uint8_t MPU6050::dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
//uint8_t MPU6050::dmpUnregisterFIFORateProcess(inv_obj_func func);
//uint8_t MPU6050::dmpRunFIFORateProcesses();

// uint8_t MPU6050::dmpSendQuaternion(uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendPacketNumber(uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
// uint8_t MPU6050::dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

uint8_t MPU6050::dmpGetAccel(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[16] << 8) | packet[17]);
    data[1] = (((uint32_t)packet[18] << 8) | packet[19]);
    data[2] = (((uint32_t)packet[20] << 8) | packet[21]);
    return 0;
}
uint8_t MPU6050::dmpGetAccel(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (packet[16] << 8) | packet[17];
    data[1] = (packet[18] << 8) | packet[19];
    data[2] = (packet[20] << 8) | packet[21];
    return 0;
}
uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    v -> x = (packet[16] << 8) | packet[17];
    v -> y = (packet[18] << 8) | packet[19];
    v -> z = (packet[20] << 8) | packet[21];
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[0] << 24) | ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3]);
    data[1] = (((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7]);
    data[2] = (((uint32_t)packet[8] << 24) | ((uint32_t)packet[9] << 16) | ((uint32_t)packet[10] << 8) | packet[11]);
    data[3] = (((uint32_t)packet[12] << 24) | ((uint32_t)packet[13] << 16) | ((uint32_t)packet[14] << 8) | packet[15]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(int16_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
    return 0;
}
uint8_t MPU6050::dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    if (status == 0) {
        q -> w = (float)qI[0] / 16384.0f;
        q -> x = (float)qI[1] / 16384.0f;
        q -> y = (float)qI[2] / 16384.0f;
        q -> z = (float)qI[3] / 16384.0f;
        return 0;
    }
    return status; // int16 return value, indicates error if this line is reached
}
// uint8_t MPU6050::dmpGet6AxisQuaternion(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetRelativeQuaternion(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGyro(int32_t *data, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    if (packet == 0) packet = dmpPacketBuffer;
    data[0] = (((uint32_t)packet[22] << 8) | packet[23]);
    data[1] = (((uint32_t)packet[24] << 8) | packet[25]);
    data[2] = (((uint32_t)packet[26] << 8) | packet[27]);
    return 0;
}
// uint8_t MPU6050::dmpSetLinearAccelFilterCoefficient(float coef);
// uint8_t MPU6050::dmpGetLinearAccel(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
    // get rid of the gravity component (+1g = +16384 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x*16384;
    v -> y = vRaw -> y - gravity -> y*16384;
    v -> z = vRaw -> z - gravity -> z*16384;
    return 0;
}
// uint8_t MPU6050::dmpGetLinearAccelInWorld(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
    return 0;
}

// uint8_t MPU6050::dmpGetGyroAndAccelSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGyroSensor(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetControlData(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetTemperature(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetGravity(long *data, const uint8_t* packet);
uint8_t MPU6050::dmpGetGravity(int16_t *data, const uint8_t* packet) {
    /* +1g corresponds to +16384, sensitivity is 2g. */
    int16_t qI[4];
    uint8_t status = dmpGetQuaternion(qI, packet);
    data[0] = ((int32_t)qI[1] * qI[3] - (int32_t)qI[0] * qI[2]) / 16384;
    data[1] = ((int32_t)qI[0] * qI[1] + (int32_t)qI[2] * qI[3]) / 16384;
    data[2] = ((int32_t)qI[0] * qI[0] - (int32_t)qI[1] * qI[1]
	       - (int32_t)qI[2] * qI[2] + (int32_t)qI[3] * qI[3]) / (int32_t)(2 * 16384L);
    return status;
}

uint8_t MPU6050::dmpGetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}
// uint8_t MPU6050::dmpGetUnquantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuantizedAccel(long *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetExternalSensorData(long *data, int size, const uint8_t* packet);
// uint8_t MPU6050::dmpGetEIS(long *data, const uint8_t* packet);

uint8_t MPU6050::dmpGetEuler(float *data, Quaternion *q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi
    return 0;
}

#ifdef USE_OLD_DMPGETYAWPITCHROLL
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
    return 0;
}
#else
uint8_t MPU6050::dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);
    if (gravity -> z < 0) {
        if(data[1] > 0) {
            data[1] = PI - data[1];
        } else {
            data[1] = -PI - data[1];
        }
    }
    return 0;
}
#endif

// uint8_t MPU6050::dmpGetAccelFloat(float *data, const uint8_t* packet);
// uint8_t MPU6050::dmpGetQuaternionFloat(float *data, const uint8_t* packet);

uint8_t MPU6050::dmpProcessFIFOPacket(const unsigned char *dmpData) {
    (void)dmpData; // unused parameter
    /*for (uint8_t k = 0; k < dmpPacketSize; k++) {
        if (dmpData[k] < 0x10) Serial.print("0");
        Serial.print(dmpData[k], HEX);
        Serial.print(" ");
    }
    Serial.print("\n");*/
    //Serial.println((uint16_t)dmpPacketBuffer);
    return 0;
}
uint8_t MPU6050::dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed) {
    uint8_t status;
    uint8_t buf[dmpPacketSize];
    for (uint8_t i = 0; i < numPackets; i++) {
        // read packet from FIFO
        getFIFOBytes(buf, dmpPacketSize);

        // process packet
        if ((status = dmpProcessFIFOPacket(buf)) > 0) return status;

        // increment external process count variable, if supplied
        if (processed != 0) (*processed)++;
    }
    return 0;
}

// uint8_t MPU6050::dmpSetFIFOProcessedCallback(void (*func) (void));

// uint8_t MPU6050::dmpInitFIFOParam();
// uint8_t MPU6050::dmpCloseFIFO();
// uint8_t MPU6050::dmpSetGyroDataSource(uint_fast8_t source);
// uint8_t MPU6050::dmpDecodeQuantizedAccel();
// uint32_t MPU6050::dmpGetGyroSumOfSquare();
// uint32_t MPU6050::dmpGetAccelSumOfSquare();
// void MPU6050::dmpOverrideQuaternion(long *q);
uint16_t MPU6050::dmpGetFIFOPacketSize() {
    return dmpPacketSize;
}



uint8_t MPU6050::dmpGetCurrentFIFOPacket(uint8_t *data) { // overflow proof
    return(GetCurrentFIFOPacket(data, dmpPacketSize));
}


*/





/*
    return false

  write-memory-block-data dataSize bank address verify/bool useProgMem/bool -> none:
      setMemoryBank(bank);
      setMemoryStartAddress(address);
      uint8_t chunkSize;
      uint8_t *verifyBuffer=0;
      uint8_t *progBuffer=0;
      uint16_t i;
      uint8_t j;
      if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
      if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
      for (i = 0; i < dataSize;) {
          // determine correct chunk size according to bank position and data size
          chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

          // make sure we don't go past the data size
          if (i + chunkSize > dataSize) chunkSize = dataSize - i;

          // make sure this chunk doesn't go past the bank boundary (256 bytes)
          if (chunkSize > 256 - address) chunkSize = 256 - address;

          if (useProgMem) {
              // write the chunk of data as specified
              for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
          } else {
              // write the chunk of data as specified
              progBuffer = (uint8_t *)data + i;
          }

          I2Cdev::writeBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, progBuffer, wireObj);

          // verify data if needed
          if (verify && verifyBuffer) {
              setMemoryBank(bank);
              setMemoryStartAddress(address);
              I2Cdev::readBytes(devAddr, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer, I2Cdev::readTimeout, wireObj);
              if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                  /*Serial.print("Block write verification error, bank ");
                  Serial.print(bank, DEC);
                  Serial.print(", address ");
                  Serial.print(address, DEC);
                  Serial.print("!\nExpected:");
                  for (j = 0; j < chunkSize; j++) {
                      Serial.print(" 0x");
                      if (progBuffer[j] < 16) Serial.print("0");
                      Serial.print(progBuffer[j], HEX);
                  }
                  Serial.print("\nReceived:");
                  for (uint8_t j = 0; j < chunkSize; j++) {
                      Serial.print(" 0x");
                      if (verifyBuffer[i + j] < 16) Serial.print("0");
                      Serial.print(verifyBuffer[i + j], HEX);
                  }
                  Serial.print("\n");*/
                  free(verifyBuffer);
                  if (useProgMem) free(progBuffer);
                  return false; // uh oh.
              }
          }

          // increase byte index by [chunkSize]
          i += chunkSize;

          // uint8_t automatically wraps to 0 at 256
          address += chunkSize;

          // if we aren't done, update bank (if necessary) and address
          if (i < dataSize) {
              if (address == 0) bank++;
              setMemoryBank(bank);
              setMemoryStartAddress(address);
          }
      }
      if (verify) free(verifyBuffer);
      if (useProgMem) free(progBuffer);
      return true;
  }
  bool MPU6050_Base::writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
      return writeMemoryBlock(data, dataSize, bank, address, verify, true);
  }
*/

  // Data found using:
  // https://github.com/thedropbears/DropBoneImu
  // https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library



  // Convert linear DMP address -> (bank, offset)
  dmp-bank_ address/int -> int:
    //BANK register only uses the low 5 bits on MPU6050 (Uses 0x1f instead of 0xFF)
    return (address >> 8) & 0x1F

  dmp-offset_ address/int -> int:
    assert: (address & 0xFF) < 256
    return address & 0xFF

  dmp-write-16-be_ address/int value/int -> none:
    hi-byte := (value >> 8) & 0xFF
    lo-byte := value & 0xFF
    dmp-write-bytes_ #[hi-byte, lo-byte] --address=address --verify

  dmp-read-16-be_ address/int -> int:
    bytes := dmp-read-bytes_ --address=address --length=2
    return i16-be_ bytes[0] bytes[1]

  dmp-write-8_ address/int value/int -> none:
    dmp-write-bytes_ #[ value & 0xFF ] --address=address --verify

  dmp-read-8_ address/int -> int:
    bytes := dmp-read-bytes_ --address=address --length=1
    return bytes[0] & 0xFF

  // Bitmask for axes the DMP tap engine should watch
  static TAP-AXIS-X-POS_ ::= 0b00000001
  static TAP-AXIS-X-NEG_ ::= 0b00000010
  static TAP-AXIS-Y-POS_ ::= 0b00000100
  static TAP-AXIS-Y-NEG_ ::= 0b00001000
  static TAP-AXIS-Z-POS_ ::= 0b00010000
  static TAP-AXIS-Z-NEG_ ::= 0b00100000

  // DMP memory map (MotionApps 6.12)
  static DMP-MEM-TAP-TIMER_     ::= 466	  // Type 16-bit BE  // Description Single-tap time window (typically µs units)
  static DMP-MEM-TAP-TH-X_      ::= 468   // Type 16-bit BE  // Description X-axis tap threshold
  static DMP-MEM-TAP-TH-Y_      ::= 472   // Type 16-bit BE  // Description Y-axis tap threshold
  static DMP-MEM-TAP-TH-Z_      ::= 476   // Type 16-bit BE  // Description Z-axis tap threshold
  static DMP-MEM-TAPW-MIN_      ::= 478   // Type 16-bit BE  // Description Multi-tap window (double/quad)
  static DMP-MEM-SH-TH-X_       ::= 436   // Type 16-bit BE  // 'Shake' thresholds (used by orient gestures)
  static DMP-MEM-SH-TH-Y_       ::= 440   // Type 16-bit BE  // 'Shake' thresholds (used by orient gestures)
  static DMP-MEM-SH-TH-Z_       ::= 444   // Type 16-bit BE  // 'Shake' thresholds (used by orient gestures)
  static DMP-MEM-ORIENT_        ::= 488   // Type 16-bit BE  // Android-orientation config/status (depends on build)
  static DMP-MEM-TAP-AXES-EN_   ::= 328   // Type 8-bit
  static DMP-MEM-TAP-DEADTIME_  ::= 474   // Type 8-bit
  static DMP-MEM-TAP-MULTI-WINDOW_  ::= 478
  static DMP-MEM-TAP-MIN-COUNT_     ::= 335
  static DMP-MEM-CFG-FIFO-ON-EVENT_ ::= 2690

  static INT-SRC-TAP_ ::= 0x01
  static INT-SRC-ANDROID-ORIENT ::= 0x08

  static DMP-MODE-CONTINUOUS_ := #[0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9]
  static DMP-MODE-GESTURE_    := #[0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda]


  enable-tap-feature -> none:
    // Make sure DMP is loaded + running, FIFO enabled, INT enabled
    // Set your feature mask (yours already has constants like DMP-FEATURE-TAP_)

    dmp-features := DMP-FEATURE-6X-LP-QUAT_ | DMP-FEATURE-TAP_  // example
    dmp-packet-size_ = dmp-packet-size_ + 4
    enable-interrupt-dmp

    set-tap-axes (TAP-AXIS-X-POS_ | TAP-AXIS-X-NEG_ | TAP-AXIS-Y-POS_ | TAP-AXIS-Y-NEG_ | TAP-AXIS-Z-POS_ | TAP-AXIS-Z-NEG_)
    set-tap-threshold (Point3i 25 25 25)    // 250 mg
    set-tap-deadtime-ms 50       // minimum time between taps on same axis
    set-tap-multi-window-ms 500   // max window to accumulate double/quad taps
    set-tap-min-count 1

  set-tap-axes mask/int -> none:
    //en := 0
    //if (mask & (TAP-AXIS-X-POS_ | TAP-AXIS-X-NEG_)) != 0: en |= 0x30
    //if (mask & (TAP-AXIS-Y-POS_ | TAP-AXIS-Y-NEG_)) != 0: en |= 0x0C
    //if (mask & (TAP-AXIS-Z-POS_ | TAP-AXIS-Z-NEG_)) != 0: en |= 0x03
    //dmp-write-8_ DMP-MEM-TAP-AXES-EN_ en

    // 6 LSBs are the +/- axis enables. First attempt was a guess so try with all on:
    dmp-write-8_ DMP-MEM-TAP-AXES-EN_ 0x3F

  // Start conservative; you can tune after first taps register
  tap-threshold-mg-to-lsb mg/int -> int:
    return clamp-value_ (mg / 32) --lower=1 --upper=255

  set-tap-threshold value-mg/Point3i -> none:
    if (value-mg.x != null): dmp-write-8_ DMP-MEM-TAP-TH-X_ (tap-threshold-mg-to-lsb value-mg.x)
    if (value-mg.y != null): dmp-write-8_ DMP-MEM-TAP-TH-Y_ (tap-threshold-mg-to-lsb value-mg.y)
    if (value-mg.z != null): dmp-write-8_ DMP-MEM-TAP-TH-Z_ (tap-threshold-mg-to-lsb value-mg.z)

  // Many builds expect microseconds in a 16-bit big-endian field.
  set-tap-deadtime-ms ms/int -> none:
    ms-ish := clamp-value_ (ms) --lower=1 --upper=255
    dmp-write-8_ DMP-MEM-TAP-DEADTIME_ ms-ish

  set-tap-multi-window-ms ms/int -> none:
    us := clamp-value_ (ms * 1000) --lower=0 --upper=65535
    dmp-write-16-be_ DMP-MEM-TAP-MULTI-WINDOW_ us

  set-tap-min-count count/int -> none:
    count = clamp-value_ count --lower=1 --upper=4
    dmp-write-8_ DMP-MEM-TAP-MIN-COUNT_ (count - 1) // zero based

  abs x/any -> any:
    if x is int:
      x < 0 ? return (x * -1) : return x
    if x is float:
      x < 0.0 ? return (x * -1.0) : return x

    logger_.error "abs: don't know how to deal with variable type." --tags={"value":x}
    throw "abs: don't know how to deal with variable type."
    return 0.0

  /**
  Read gesture information from one packet.

  read-tap-status packet/ByteArray?=null -> int:
    if packet == null: packet = read-dmp-packet --drain
    if packet == null: return  0  // avoid crash
    [28,29,30,31]
    tapped := dmp-read-8_ DMP-MEM-TAP-STATUS_
    count := dmp-read-8_ DMP-MEM-TAP-COUNT_
    if not (tapped == 1):
      return 0
    else:
      return count
  */

  /**
  Sets FIFO to recieve one packet per elapsed period (set by dmp-set-sample-rate).
  */
  dmp-set-interrupt-mode-continuous -> none:
    dmp-write-bytes_ DMP-MODE-CONTINUOUS_ --address=DMP-MEM-CFG-FIFO-ON-EVENT_

  /**
  Sets FIFO to one per gesture.
  */
  dmp-set-interrupt-mode-gesture -> none:
    dmp-write-bytes_ DMP-MODE-GESTURE_ --address=DMP-MEM-CFG-FIFO-ON-EVENT_



/*
  handle-int -> none:
    st := read-register_ Mpu6050.REG-INTRPT-STATUS_ --width=8

    // DMP interrupt bit set?
    if (st & INTRPT-DMP-MASK_) != 0:
      // 1) See if there’s a FIFO data packet (your normal drain path)
      count := read-register_ Mpu6050.REG-FIFO-COUNT_ --width=16
      if count >= DMP-PACKET-SIZE_ && (count % DMP-PACKET-SIZE_) == 0:
        // drain packets as usual (quaternion/accel/gyro)
        _ := read-dmp-packet --drain=true

      // 2) Regardless, check for a tap event (cheap)
      dir,count2 := read-tap-status
      if count2 > 0 && on-tap-cb != null:
        axis,sign := decode-tap-dir dir
        on-tap-cb axis sign count2


  decode-tap-dir d/int -> (char, int):
    // Convention (verify once): bit0=+X, bit1=-X, bit2=+Y, bit3=-Y, bit4=+Z, bit5=-Z
    if (d & (1<<0)) != 0: return ('x', +1)
    if (d & (1<<1)) != 0: return ('x', -1)
    if (d & (1<<2)) != 0: return ('y', +1)
    if (d & (1<<3)) != 0: return ('y', -1)
    if (d & (1<<4)) != 0: return ('z', +1)
    if (d & (1<<5)) != 0: return ('z', -1)
    return ('x', 0)  // none

*/


  // Compute gravity unit vector (in body frame) from a unit quaternion q = (w,x,y,z).
  gravity-from-quaternion q/Vector4f -> Point3f:
    w := q.x     // if your Vector4f is (w,x,y,z): q.x is w, q.y is x, etc.
    x := q.y
    y := q.z
    z := q.w

    // Optional: normalize once, in case q isn't strictly unit length.
    norm2 := w*w + x*x + y*y + z*z
    if (norm2 != 0.0) and ((abs (norm2 - 1.0)) > 1e-5):
      inv := 1.0 / (sqrt norm2)
      w *= inv
      x *= inv
      y *= inv
      z *= inv

    gx := 2.0 * (x*z - w*y)
    gy := 2.0 * (w*x + y*z)
    gz := w*w - x*x - y*y + z*z
    return Point3f gx gy gz
