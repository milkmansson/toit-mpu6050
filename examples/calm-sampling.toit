// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
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

  print " get-whoami returned: 0x$(%02x driver.get-whoami)"
  print " get-temperature returned: $(%0.3f driver.read-temperature)c"
