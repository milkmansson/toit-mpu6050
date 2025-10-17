// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.   This file also includes derivative
// work from other authors and sources with permission.  See README.md.

import log
import math
import binary
import serial.device as serial
import serial.registers as registers


class Mpu6050:
  /**  Default $I2C-ADDRESS is 0x36  */
  static I2C-ADDRESS ::= 0x68
  static MPU-6050-WHOAMI ::= 0x34

  static DEFAULT-REGISTER-WIDTH_ ::= 16 // bits


  static REG-ACCEL-XOUT_ ::= 0x3b
  static REG-ACCEL-YOUT_ ::= 0x3d
  static REG-ACCEL-ZOUT_ ::= 0x3f
  static REG-ACCEL-CONFIG_ ::= 0x1c
  static ACCEL-FS-SELECT-MASK_  ::= 0b00011000
  static ACCEL-X-SELFTEST-MASK_ ::= 0b10000000
  static ACCEL-Y-SELFTEST-MASK_ ::= 0b01000000
  static ACCEL-Z-SELFTEST-MASK_ ::= 0b00100000

  static REG-GYRO-XOUT_ ::= 0x43
  static REG-GYRO-YOUT_ ::= 0x45
  static REG-GYRO-ZOUT_ ::= 0x47
  static REG-GYRO-CONFIG_ ::= 0x1b
  static GYRO-FS-SELECT-MASK_ ::= 0b00011000
  static GYRO-X-SELFTEST-MASK_ ::= 0b10000000
  static GYRO-Y-SELFTEST-MASK_ ::= 0b01000000
  static GYRO-Z-SELFTEST-MASK_ ::= 0b00100000

  // Temperature
  static REG-TEMP_ ::= 0x41
  static TEMP-SO_      ::= 340.0
  static TEMP-OFFSET_  ::= 36.53

  static REG-WHO-AM-I_ ::= 0x75
  static REG-WHO-AM-I-MASK_ ::= 0b01111110

  // Power Management
  static REG-POWER-MANAGEMENT_ ::= 0x6b
  static PM-DEVICE-RESET-MASK_ ::= 0b10000000_00000000
  static PM-SLEEP-MASK_        ::= 0b01000000_00000000
  static PM-CYCLE-MASK_        ::= 0b00100000_00000000
  static PM-TEMP-DISABLE-MASK_ ::= 0b00001000_00000000
  static PM-CLOCK-SOURCE-MASK_ ::= 0b00000111_00000000
  static PM-LP-WAKE-CTRL-MASK_ ::= 0b00000000_11000000
  static PM-STBY-ACCEL-X-MASK_ ::= 0b00000000_00100000
  static PM-STBY-ACCEL-Y-MASK_ ::= 0b00000000_00010000
  static PM-STBY-ACCEL-Z-MASK_ ::= 0b00000000_00001000
  static PM-STBY-GYRO-X-MASK_  ::= 0b00000000_00000100
  static PM-STBY-GYRO-Y-MASK_  ::= 0b00000000_00000010
  static PM-STBY-GYRO-Z-MASK_  ::= 0b00000000_00000001

  // Signal Path Reset
  static REG-SIGNAL-PATH-RESET_ ::= 0x68
  static SIGNAL-PATH-GYRO-RESET-MASK_  ::= 0b00000100
  static SIGNAL-PATH-ACCEL-RESET-MASK_ ::= 0b00000010
  static SIGNAL-PATH-TEMP-RESET-MASK_  ::= 0b00000001

  // Globals
  reg_/registers.Registers := ?
  logger_/log.Logger := ?

  /** Class Constructor:  */
  constructor
      device/serial.Device
      --logger/log.Logger=log.default:
    logger_ = logger.with-name "mpu60x0"
    reg_ = device.registers

    tries := 5
    while get-whoami != MPU-6050-WHOAMI:
      tries -= 1
      if tries == 0:
        logger_.error "Device is NOT an MPU60x0" --tags={ "expected-id" : MPU-6050-WHOAMI, "received-id": get-whoami}

    // Reset: Destabilises I2C bus - ignoring for now.
    sleep --ms=100  // Sleep 100ms to stabilize.

    // Init: Device is asleep at power on
    wakeup-now
    reset-gyro
    reset-temp
    reset-accel
    enable-temperature
    set-clock-source

  /**
  Resets the device.
  */
  reset -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-DEVICE-RESET-MASK_ --width=16

  /**
  Resets Gyroscope Signal Path.
  */
  reset-gyro -> none:
    write-register_ REG-SIGNAL-PATH-RESET_ 1 --mask=SIGNAL-PATH-GYRO-RESET-MASK_ --width=8

  /**
  Resets Temperature Signal Path.
  */
  reset-temp -> none:
    write-register_ REG-SIGNAL-PATH-RESET_ 1 --mask=SIGNAL-PATH-TEMP-RESET-MASK_ --width=8

  /**
  Resets Accelerometer Signal Path.
  */
  reset-accel -> none:
    write-register_ REG-SIGNAL-PATH-RESET_ 1 --mask=SIGNAL-PATH-ACCEL-RESET-MASK_ --width=8

  /**
  Enable Temperature Measurements.
  */
  enable-temperature -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-TEMP-DISABLE-MASK_ --width=16

  /**
  Disable Temperature Measurements.
  */
  disable-temperature -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-TEMP-DISABLE-MASK_ --width=16

  /**
  Goes to sleep.
  */
  sleep-now -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-SLEEP-MASK_ --width=16

  /**
  Wakes up.
  */
  wakeup-now -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-SLEEP-MASK_ --width=16

  /**
  Sets Clock Source.

  CLKSEL Clock Source
    0 Internal 8MHz oscillator
    1 PLL with X axis gyroscope reference
    2 PLL with Y axis gyroscope reference
    3 PLL with Z axis gyroscope reference
    4 PLL with external 32.768kHz reference
    5 PLL with external 19.2MHz reference
    6 Reserved
    7 Stops the clock and keeps the timing generator in reset
  */
  set-clock-source source/int=0 -> none:
    write-register_ REG-POWER-MANAGEMENT_ source --mask=PM-CLOCK-SOURCE-MASK_ --width=16


  /**
  Reads acceleration at this moment.

  Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
  (Register 28). For each full scale setting, the accelerometers’ sensitivity
  per LSB in ACCEL_xOUT is shown in the table below.
  AFS_SEL Full Scale Range LSB Sensitivity
  - 0 ±2g 16384 LSB/g
  - 1 ±4g 8192 LSB/g
  - 2 ±8g 4096 LSB/g
  - 3 ±16g 2048 LSB/g

  */
  read-acceleration -> math.Point3f:
    a-fs := get-accel-fs
    a-lsb := convert-accel-fs-to-scale_ a-fs
    a-x := (read-register_ REG-ACCEL-XOUT_ --signed) * a-lsb
    a-y := (read-register_ REG-ACCEL-YOUT_ --signed) * a-lsb
    a-z := (read-register_ REG-ACCEL-ZOUT_ --signed) * a-lsb
    return math.Point3f a-x a-y a-z

  /**
  Gets Accelerometer scale value from the register.
  */
  get-accel-fs -> int:
    return read-register_ REG-ACCEL-CONFIG_ --mask=ACCEL-FS-SELECT-MASK_ --width=8

  convert-accel-fs-to-scale_ value/int -> int:
    // AFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= value <= 3
    if value == 0: return 16384 // ±2g 16384 LSB/g
    if value == 1: return 8192  // ±4g 8192 LSB/g
    if value == 2: return 4096  // ±8g 4096 LSB/g
    if value == 3: return 2048  // ±16g 2048 LSB/g
    return 0

  execute-accel-selftest-x -> none:
    write-register_ REG-ACCEL-CONFIG_ 1 --mask=ACCEL-X-SELFTEST-MASK_ --width=8

  execute-accel-selftest-y -> none:
    write-register_ REG-ACCEL-CONFIG_ 1 --mask=ACCEL-Y-SELFTEST-MASK_ --width=8

  execute-accel-selftest-z -> none:
    write-register_ REG-ACCEL-CONFIG_ 1 --mask=ACCEL-Z-SELFTEST-MASK_ --width=8



  /**
  Reads gyroscopic orientation at this moment.

  Each 16-bit gyroscope measurement has a full scale defined in FS_SEL (Register
  27). For each full scale setting, the gyroscopes’ sensitivity per LSB in GYRO
  xOUT is shown in the table below:
  FS_SEL Full Scale Range LSB Sensitivity
  - 0 ± 250 °/s 131 LSB/°/s
  - 1 ± 500 °/s 65.5 LSB/°/s
  - 2 ± 1000 °/s 32.8 LSB/°/s
  - 3 ± 2000 °/s 16.4 LSB/°/s
  */
  read-gyro -> math.Point3f:
    g-fs := get-gyro-fs
    g-lsb := convert-gyro-fs-to-scale_ g-fs
    g-x := (read-register_ REG-GYRO-XOUT_ --signed) / g-lsb
    g-y := (read-register_ REG-GYRO-YOUT_ --signed) / g-lsb
    g-z := (read-register_ REG-GYRO-ZOUT_ --signed) / g-lsb
    return math.Point3f g-x g-y g-z

  /**
  Gets Gyroscope scale values from the register.
  */
  get-gyro-fs -> int:
    return read-register_ REG-GYRO-CONFIG_ --mask=GYRO-FS-SELECT-MASK_ --width=8

  convert-gyro-fs-to-scale_ value/int -> float:
    // AFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= value <= 3
    if value == 0: return 131.0 // ± 250  °/s 131 LSB/°/s
    if value == 1: return 65.5  // ± 500  °/s 65.5 LSB/°/s
    if value == 2: return 32.8  // ± 1000 °/s 32.8 LSB/°/s
    if value == 3: return 16.4  // ± 2000 °/s 16.4 LSB/°/s
    return 0.0

  execute-gyro-selftest-x -> none:
    write-register_ REG-GYRO-CONFIG_ 1 --mask=GYRO-X-SELFTEST-MASK_ --width=8

  execute-gyro-selftest-y -> none:
    write-register_ REG-GYRO-CONFIG_ 1 --mask=GYRO-X-SELFTEST-MASK_ --width=8

  execute-gyro-selftest-z -> none:
    write-register_ REG-GYRO-CONFIG_ 1 --mask=GYRO-X-SELFTEST-MASK_ --width=8

  read-temperature -> float:
    raw := read-register_ REG-TEMP_ --signed
    return ((raw.to-float / TEMP-SO_) + TEMP-OFFSET_)

  /**
  Returns the value of the WHO_AM_I register.

  The MPU-6050 should return 0x34.
  */
  get-whoami -> int:
    return read-register_ REG-WHO-AM-I_ --mask=REG-WHO-AM-I-MASK_ --width=8

  read-register_
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> any:
    assert: (width == 8) or (width == 16)
    if mask == null:
      mask = (width == 16) ? 0xFFFF : 0xFF
    if offset == null:
      offset = mask.count-trailing-zeros

    register-value/int? := null
    if width == 8:
      if signed:
        register-value = reg_.read-i8 register
      else:
        register-value = reg_.read-u8 register
    if width == 16:
      if signed:
        register-value = reg_.read-i16-be register
      else:
        register-value = reg_.read-u16-be register

    if register-value == null:
      logger_.error "read-register_: Read failed."
      throw "read-register_: Read failed."

    if ((mask == 0xFFFF) or (mask == 0xFF)) and (offset == 0):
      return register-value
    else:
      masked-value := (register-value & mask) >> offset
      return masked-value

  write-register_
      register/int
      value/any
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false -> none:
    assert: (width == 8) or (width == 16)
    if mask == null:
      mask = (width == 16) ? 0xFFFF : 0xFF
    if offset == null:
      offset = mask.count-trailing-zeros

    field-mask/int := (mask >> offset)
    assert: ((value & ~field-mask) == 0)  // fit check

    // Full-width direct write
    if ((width == 8)  and (mask == 0xFF)  and (offset == 0)) or
      ((width == 16) and (mask == 0xFFFF) and (offset == 0)):
      if width == 8:
        signed ? reg_.write-i8 register (value & 0xFF) : reg_.write-u8 register (value & 0xFF)
      else:
        signed ? reg_.write-i16-be register (value & 0xFFFF) : reg_.write-u16-be register (value & 0xFFFF)
      return

    // Read Reg for modification
    old-value/int? := null
    if width == 8:
      if signed :
        old-value = reg_.read-i8 register
      else:
        old-value = reg_.read-u8 register
    else:
      if signed :
        old-value = reg_.read-i16-be register
      else:
        old-value = reg_.read-u16-be register

    if old-value == null:
      logger_.error "write-register_: Read existing value (for modification) failed."
      throw "write-register_: Read failed."

    new-value/int := (old-value & ~mask) | ((value & field-mask) << offset)

    if width == 8:
      signed ? reg_.write-i8 register new-value : reg_.write-u8 register new-value
      return
    else:
      signed ? reg_.write-i16-be register new-value : reg_.write-u16-be register new-value
      return

    throw "write-register_: Unhandled Circumstance."

  /**
  Clamps the supplied value to specified limit.
  */
  clamp-value_ value/any --upper/any?=null --lower/any?=null -> any:
    if upper != null: if value > upper:  return upper
    if lower != null: if value < lower:  return lower
    return value

  /**
  Provides strings to display bitmasks nicely when testing.
  */
  bits-16_ x/int --min-display-bits/int=0 -> string:
    assert: (x >= 0) and (x <= 0xFFFF)
    if (x > 255) or (min-display-bits > 8):
      out-string := "$(%b x)"
      out-string = out-string.pad --left 16 '0'
      out-string = "$(out-string[0..4]).$(out-string[4..8]).$(out-string[8..12]).$(out-string[12..16])"
      return out-string
    else if (x > 15) or (min-display-bits > 4):
      out-string := "$(%b x)"
      out-string = out-string.pad --left 8 '0'
      out-string = "$(out-string[0..4]).$(out-string[4..8])"
      return out-string
    else:
      out-string := "$(%b x)"
      out-string = out-string.pad --left 4 '0'
      out-string = "$(out-string[0..4])"
      return out-string
