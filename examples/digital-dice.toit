
// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import math
import mpu6050 show *

sda-pin-number := 19    // please set these correctly for your device
scl-pin-number := 20    // please set these correctly for your device

main:
  // Enable and drive I2C:
  frequency := 400_000
  sda-pin := gpio.Pin sda-pin-number
  scl-pin := gpio.Pin scl-pin-number
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=frequency

  if not bus.test Mpu6050.I2C_ADDRESS:
    print " No Mpu60x0 device found"
    return

  print " Found Mpu60x0 on 0x$(%02x Mpu6050.I2C_ADDRESS)"
  device := bus.device Mpu6050.I2C_ADDRESS
  driver := Mpu6050 device

  // sens argument configures wake up sensitivity

  writeByte( MPU6050_ADDRESS, 0x6B, 0x00);
  writeByte( MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0x07);  // Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
  // writeByte( MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20);   // write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
  writeByte( MPU6050_ADDRESS, ACCEL_CONFIG, 0x01);       // Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
  writeByte( MPU6050_ADDRESS, MOT_THR, sens);            // Write the desired Motion threshold to register 0x1F (For example, write decimal 20).
  writeByte( MPU6050_ADDRESS, MOT_DUR, 40 );             // Set motion detect
  //duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  writeByte( MPU6050_ADDRESS, MOT_DETECT_CTRL, 0x15);    // to register 0x69,
  //write the motion detection decrement and a few other settings (for example
  //write 0x15 to set both free-fall and motion decrements to 1 and accelerometer
  //start-up delay to 5ms total by adding 1ms. )



  // Set interrupt pin to go low when activated
  // writeByte( MPU6050_ADDRESS, 0x37, 140 ); // now INT pin is active low
  driver.set-interrupt-pin-active-low
  driver.set-interrupt-fsync-pin-active-high
  driver.enable-fsync-pin

  // Enable motion detection interrupt
  driver.enable-data-ready-interrupt

  // Disable Temperature
  driver.disable-temperature

  // Disable Gyroscope
  driver.disable-gyroscope
