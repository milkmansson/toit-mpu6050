
// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import math
import mpu6050 show *

sda-pin-number := 19    // please set these correctly for your device
scl-pin-number := 20    // please set these correctly for your device
interrupt-pin-number := 6

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

  // Configure Interrupt Pin
  interrupt-pin := gpio.Pin interrupt-pin-number --input --pull-up

  // Configure Digital High Pass Filter
  driver.set-accelerometer-high-pass-filter Mpu6050.ACCEL-HPF-5HZ
  driver.set-clock-source Mpu6050.CLOCK-SRC-INTERNAL-8MHZ
  driver.wakeup-now

  // writeByte( MPU6050_ADDRESS, I2C_SLV0_ADDR, 0x20);   // write register 0x37
  //to select how to use the interrupt pin. For an active high, push-pull signal
  //that stays until register (decimal) 58 is read, write 0x20.

  // Reset all internal signal paths
  driver.reset-gyroscope
  driver.reset-accelerometer
  driver.reset-temperature

  // Set Motion Detection Duration
  // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  driver.set-motion-detection-duration 40

  // Set Motion Detection Threshold: wake up sensitivity.
  driver.set-motion-detection-threshold 1

  // Set decrement rates and delay for freefall and motion detection
  driver.set-free-fall-count-decrement-rate 1
  driver.set-motion-detection-count-decrement-rate 1
  driver.set-acceleration-wake-delay-ms 5

  // Set interrupt pin to go low when activated (original wrote 140 to 0x37)
  driver.set-interrupt-pin-active-low
  driver.set-interrupt-fsync-pin-active-high
  driver.enable-fsync-pin

  // Enable motion detection interrupt
  driver.enable-data-ready-interrupt

  // Disable Temperature (unused in this instance)
  driver.disable-temperature

  // Disable Gyroscope (unused in this instance)
  driver.disable-gyroscope

  // At this point we wait....  When the motion detection triggers, the pin
  // will activate and we go get a value.
  new-accel := ?
  while true:
    interrupt-pin.wait-for 0
    new-accel = driver.read-acceleration
    print " read-accel returned: $(%0.4f new-accel.x)x.g $(%0.4f new-accel.y)y.g $(%0.4f new-accel.z)z.g"
    sleep --ms=100
