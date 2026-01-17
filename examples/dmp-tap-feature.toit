// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import math
import mpu6050.mpu6050-dmp-ma612 show *

sda-pin-number       := 19 // please set these correctly for your device
scl-pin-number       := 20 // please set these correctly for your device
interrupt-pin-number := 4  // please set these correctly for your device

main:
  // Enable and drive I2C:
  frequency := 400_000
  sda-pin := gpio.Pin sda-pin-number
  scl-pin := gpio.Pin scl-pin-number
  bus := i2c.Bus --sda=sda-pin --scl=scl-pin --frequency=frequency

  if not bus.test Mpu6050-dmp-ma612.I2C_ADDRESS:
    print " No Mpu60x0 device found"
    return

  print " Found Mpu60x0 on 0x$(%02x Mpu6050-dmp-ma612.I2C_ADDRESS)"
  device := bus.device Mpu6050-dmp-ma612.I2C_ADDRESS
  driver := Mpu6050-dmp-ma612 device

  // Configure Interrupt Pin, Defaults, and wake MPU6050
  interrupt-pin := gpio.Pin interrupt-pin-number --input --pull-down
  driver.set-clock-source Mpu6050-dmp-ma612.CLOCK-SRC-INTERNAL-8MHZ
  driver.set-interrupt-pin-active-high
  driver.disable-fsync-pin
  driver.wakeup-now

  // Reset all internal signal paths
  driver.reset-gyroscope
  driver.reset-accelerometer
  driver.reset-temperature

  // Disable Unused Bits
  driver.disable-temperature

  // Set interrupt pin activity
  driver.set-interrupt-pin-active-low

  // Enables the driver, uploads MA code to mpu6050
  print "Enabling DMP then waiting....."
  driver.enable-dmp
  sleep --ms=2000

  // Enables the tap feature
  print "Enabling Tap Feature then waiting....."
  driver.enable-tap-feature
  sleep --ms=2000

  // Converts FIFO to tap events only
  print "Setting Tap Interrupt then waiting....."
  driver.dmp-set-interrupt-mode-gesture
  //driver.dmp-set-interrupt-mode-continuous
  sleep --ms=2000

  // Clear the buffer of old (pre tap-enabled) events
  driver.clear-fifo

  // Repeat (insted)
  100.repeat:
    print "$(it) buffer $(driver.buffer-size)"
    sleep --ms=1000
