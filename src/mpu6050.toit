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
  static I2C-ADDRESS-AD0-0 ::= 0x68
  static I2C-ADDRESS-AD0-1 ::= 0x69
  static MPU-6050-WHOAMI ::= 0x34

  static DEFAULT-REGISTER-WIDTH_ ::= 16 // bits

  static REG-ACCEL-XOUT_ ::= 0x3b
  static REG-ACCEL-YOUT_ ::= 0x3d
  static REG-ACCEL-ZOUT_ ::= 0x3f
  static REG-ACCEL-CONFIG_ ::= 0x1c
  static ACCEL-X-SELFTEST-MASK_ ::= 0b10000000
  static ACCEL-Y-SELFTEST-MASK_ ::= 0b01000000
  static ACCEL-Z-SELFTEST-MASK_ ::= 0b00100000
  static ACCEL-FS-SELECT-MASK_  ::= 0b00011000
  static GYRO-FS-131-0 ::= 0  // 131 LSB/deg/s  +/- 250deg/s
  static GYRO-FS-65-5  ::= 1  // 65.5 LSB/deg/s  +/- 250deg/s
  static GYRO-FS-32-8  ::= 2  // 32.8 LSB/deg/s  +/- 250deg/s
  static GYRO-FS-16-4  ::= 3  // 16.4 LSB/deg/s  +/- 250deg/s

  static REG-GYRO-XOUT_ ::= 0x43
  static REG-GYRO-YOUT_ ::= 0x45
  static REG-GYRO-ZOUT_ ::= 0x47
  static REG-GYRO-CONFIG_ ::= 0x1b
  static GYRO-X-SELFTEST-MASK_ ::= 0b10000000
  static GYRO-Y-SELFTEST-MASK_ ::= 0b01000000
  static GYRO-Z-SELFTEST-MASK_ ::= 0b00100000
  static GYRO-FS-SELECT-MASK_ ::= 0b00011000
  static ACCEL-FS-RANGE-2G  ::= 0  // 16384 LSB/g  +/- 2g
  static ACCEL-FS-RANGE-4G  ::= 1  // 8192 LSB/g  +/- 4g
  static ACCEL-FS-RANGE-8G  ::= 2  // 4096 LSB/g  +/- 8g
  static ACCEL-FS-RANGE-16G ::= 3  // 2048 LSB/g  +/- 16g

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

  // Interrupts
  static REG-INTRPT-ENABLE_       ::= 0x38
  static INTRPT-DATA-READY-MASK_    ::= 0b00000001
  static INTRPT-I2C-MASTER-SOURCES_ ::= 0b00001000
  static INTRPT-FIFO-OVERFLOW_      ::= 0b00010000

  static REG-INTRPT-PIN-CONFIG_ ::= 0x37
  static INTRPT-PIN-ACTIVE-HL_       ::= 0b10000000  // 0 active high or 1 active low
  static INTRPT-PIN-PUSH-OPEN_       ::= 0b01000000  // 0 Push Pull or 1 Open Drain
  static INTRPT-PIN-LATCH_           ::= 0b00100000  // 0 50us pulse or 1 latch
  static INTRPT-READ-CLEAR_          ::= 0b00010000  // 0 manual clearing or 1 reads clear int
  static INTRPT-FSYNC-PIN-ACTIVE-HL_ ::= 0b00001000
  static INTRPT-FSYNC-PIN-ENABLE_    ::= 0b00000100
  static INTRPT-I2C-BYPASS-ENABLE_   ::= 0b00000010


  // Undocumented
  static MOT-DETECT-THR_      ::= 0x1F  // Motion detection threshold bits [7:0]
  static MOT-DETECT-DURATION_ ::= 0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
  static MOT-DETECT_CTRL_     ::= 0x69


  // Experimental
  static deg_/float ::= (180.0 / math.PI)
  static eps_/float ::= 1e-9

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

    // Init: Device is asleep at power on, returning 0 for all values
    wakeup-now
    reset-gyro
    reset-temp
    reset-accel
    enable-temperature
    set-clock-source
    set-accel-fs ACCEL-FS-RANGE-8G  // Set accelerometer range to +/- 8g
    set-gyro-fs GYRO-FS-131-0


  /**
  Resets the device.

  Note: I2C bus may become unstable, as the device seems to reset and does not
  seem to quite complete the I2C transaction
  */
  reset -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-DEVICE-RESET-MASK_ --width=16
    sleep --ms=200

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

  Each 16-bit accelerometer measurement has a full scale defined in
  $ACCEL-FS-SELECT-MASK_ in register $REG-ACCEL-CONFIG_. For each full scale
  setting, the accelerometers’ sensitivity changes per LSB:
  SELECTION | Full Scale Range LSB Sensitivity
  - 0 | ±2g 16384 LSB/g
  - 1 | ±4g 8192 LSB/g
  - 2 | ±8g 4096 LSB/g
  - 3 | ±16g 2048 LSB/g
  */
  read-acceleration -> math.Point3f:
    a-fs := get-accel-fs   // Obtain range configuration
    a-lsb := convert-accel-fs-to-scale_ a-fs  // Obtain multiplier
    a-x := (read-register_ REG-ACCEL-XOUT_ --signed).to-float / a-lsb
    a-y := (read-register_ REG-ACCEL-YOUT_ --signed).to-float / a-lsb
    a-z := (read-register_ REG-ACCEL-ZOUT_ --signed).to-float / a-lsb
    return math.Point3f a-x a-y a-z

  /**
  Reads acceleration at this moment, returning raw register reads.
  */
  read-raw-acceleration -> math.Point3f:
    a-x := read-register_ REG-ACCEL-XOUT_ --signed
    a-y := read-register_ REG-ACCEL-YOUT_ --signed
    a-z := read-register_ REG-ACCEL-ZOUT_ --signed
    return math.Point3f a-x a-y a-z

  /**
  Gets Accelerometer scale value from the register.
  */
  get-accel-fs -> int:
    return read-register_ REG-ACCEL-CONFIG_ --mask=ACCEL-FS-SELECT-MASK_ --width=8

  /**
  Sets Accelerometer scale values from the register.
  */
  set-accel-fs raw/int -> none:
    assert: 0 <= raw <= 3
    write-register_ REG-ACCEL-CONFIG_ raw --mask=ACCEL-FS-SELECT-MASK_ --width=8

  /**
  Converts Accelerometer scale selectors to actual values/multipliers.
  */
  convert-accel-fs-to-scale_ value/int -> int:
    // AFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= value <= 3
    if value == ACCEL-FS-RANGE-2G: return 16384 // ±2g 16384 LSB/g
    if value == ACCEL-FS-RANGE-4G: return 8192  // ±4g 8192 LSB/g
    if value == ACCEL-FS-RANGE-8G: return 4096  // ±8g 4096 LSB/g
    if value == ACCEL-FS-RANGE-16G: return 2048  // ±16g 2048 LSB/g
    return 0

  execute-accel-selftest -> none:
    execute-accel-selftest-x
    execute-accel-selftest-y
    execute-accel-selftest-z

  execute-accel-selftest-x -> none:
    write-register_ REG-ACCEL-CONFIG_ 1 --mask=ACCEL-X-SELFTEST-MASK_ --width=8

  execute-accel-selftest-y -> none:
    write-register_ REG-ACCEL-CONFIG_ 1 --mask=ACCEL-Y-SELFTEST-MASK_ --width=8

  execute-accel-selftest-z -> none:
    write-register_ REG-ACCEL-CONFIG_ 1 --mask=ACCEL-Z-SELFTEST-MASK_ --width=8

  enable-accelerometer -> none:
    enable-accelerometer-x
    enable-accelerometer-y
    enable-accelerometer-z

  disable-accelerometer -> none:
    disable-accelerometer-x
    disable-accelerometer-y
    disable-accelerometer-z

  enable-accelerometer-x -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-STBY-ACCEL-X-MASK_

  enable-accelerometer-y -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-STBY-ACCEL-Y-MASK_

  enable-accelerometer-z -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-STBY-ACCEL-Z-MASK_

  disable-accelerometer-x -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-STBY-ACCEL-X-MASK_

  disable-accelerometer-y -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-STBY-ACCEL-Y-MASK_

  disable-accelerometer-z -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-STBY-ACCEL-Z-MASK_

  /**
  Sets Interrupts
  */
  enable-data-ready-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-DATA-READY-MASK_ --width=8

  disable-data-ready-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-DATA-READY-MASK_ --width=8

  enable-i2c-master-sources-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-I2C-MASTER-SOURCES_ --width=8

  disable-i2c-master-sources-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-I2C-MASTER-SOURCES_ --width=8

  enable-fifo-overflow-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-FIFO-OVERFLOW_ --width=8

  disable-fifo-overflow-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-FIFO-OVERFLOW_ --width=8

  set-interrupt-pin-active-high -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-PIN-ACTIVE-HL_ --width=8

  set-interrupt-pin-active-low -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-PIN-ACTIVE-HL_ --width=8

  set-interrupt-pin-push-pull -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-PIN-PUSH-OPEN_ --width=8

  set-interrupt-pin-open-drain -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-PIN-PUSH-OPEN_ --width=8

  set-interrupt-pin-manual-clear -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-READ-CLEAR_ --width=8

  set-interrupt-pin-read-clears -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-READ-CLEAR_ --width=8

  set-interrupt-fsync-pin-active-high -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-FSYNC-PIN-ACTIVE-HL_ --width=8

  set-interrupt-fsync-pin-active-low -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-FSYNC-PIN-ACTIVE-HL_ --width=8

  enable-fsync-pin -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-FSYNC-PIN-ENABLE_ --width=8

  disable-fsync-pin -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-FSYNC-PIN-ENABLE_ --width=8

  /**
  Enables I2C bypass.

  When this is enabled (and I2C_MST_EN (Register 106 bit[5]) is equal to 0) the
   host application processor will be able to directly access the auxiliary I2C
   bus of the MPU-60X0.
  */
  enable-i2c-bypass -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-I2C-BYPASS-ENABLE_ --width=8

  /**
  Disables I2C bypass.

  When this is not enabled, the host application processor will not be able to
  directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state
  of I2C_MST_EN (Register 106 bit[5]).
  */
  disable-i2c-bypass -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-I2C-BYPASS-ENABLE_ --width=8
















  /**
  Reads gyroscopic orientation at this moment.

  Each 16-bit gyroscope measurement has a full scale defined in FS_SEL (Register
  27). For each full scale setting, the gyroscopes’ sensitivity per LSB in GYRO
  xOUT is shown in the table below:
  SELECTION | Full Scale Range LSB Sensitivity
  - 0 | ± 250 °/s 131 LSB/°/s
  - 1 | ± 500 °/s 65.5 LSB/°/s
  - 2 | ± 1000 °/s 32.8 LSB/°/s
  - 3 | ± 2000 °/s 16.4 LSB/°/s
  */
  read-gyroscope -> math.Point3f:
    g-fs := get-gyro-fs   // Obtain range configuration
    g-lsb := convert-gyro-fs-to-scale_ g-fs   // Obtain multiplier
    g-x := (read-register_ REG-GYRO-XOUT_ --signed) / g-lsb
    g-y := (read-register_ REG-GYRO-YOUT_ --signed) / g-lsb
    g-z := (read-register_ REG-GYRO-ZOUT_ --signed) / g-lsb
    return math.Point3f g-x g-y g-z

  /**
  Gets Gyroscope scale values from the register.
  */
  get-gyro-fs -> int:
    return read-register_ REG-GYRO-CONFIG_ --mask=GYRO-FS-SELECT-MASK_ --width=8

  /**
  Sets Gyroscope scale values from the register.
  */
  set-gyro-fs raw/int -> none:
    assert: 0 <= raw <= 3
    write-register_ REG-GYRO-CONFIG_ raw --mask=GYRO-FS-SELECT-MASK_ --width=8

  /**
  Converts Gyroscope scale selectors to actual values/multipliers.
  */
  convert-gyro-fs-to-scale_ value/int -> float:
    // GFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= value <= 3
    if value == GYRO-FS-131-0: return 131.0 // ± 250  °/s 131 LSB/°/s
    if value == GYRO-FS-65-5:  return 65.5  // ± 500  °/s 65.5 LSB/°/s
    if value == GYRO-FS-32-8:  return 32.8  // ± 1000 °/s 32.8 LSB/°/s
    if value == GYRO-FS-16-4:  return 16.4  // ± 2000 °/s 16.4 LSB/°/s
    return 0.0

  execute-gyro-selftest -> none:
    execute-gyro-selftest-x
    execute-gyro-selftest-y
    execute-gyro-selftest-z

  execute-gyro-selftest-x -> none:
    write-register_ REG-GYRO-CONFIG_ 1 --mask=GYRO-X-SELFTEST-MASK_ --width=8

  execute-gyro-selftest-y -> none:
    write-register_ REG-GYRO-CONFIG_ 1 --mask=GYRO-Y-SELFTEST-MASK_ --width=8

  execute-gyro-selftest-z -> none:
    write-register_ REG-GYRO-CONFIG_ 1 --mask=GYRO-Z-SELFTEST-MASK_ --width=8

  read-temperature -> float:
    raw := read-register_ REG-TEMP_ --signed
    return ((raw.to-float / TEMP-SO_) + TEMP-OFFSET_)

  enable-gyroscope -> none:
    enable-gyroscope-x
    enable-gyroscope-y
    enable-gyroscope-z

  disable-gyroscope -> none:
    disable-gyroscope-x
    disable-gyroscope-y
    disable-gyroscope-z

  enable-gyroscope-x -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-STBY-GYRO-X-MASK_

  enable-gyroscope-y -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-STBY-GYRO-Y-MASK_

  enable-gyroscope-z -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-STBY-GYRO-Z-MASK_

  disable-gyroscope-x -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-STBY-GYRO-X-MASK_

  disable-gyroscope-y -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-STBY-GYRO-Y-MASK_

  disable-gyroscope-z -> none:
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-STBY-GYRO-Z-MASK_

  /**
  Returns the value of the WHO_AM_I register.

  The MPU-6050 should return 0x34.
  */
  get-whoami -> int:
    return read-register_ REG-WHO-AM-I_ --mask=REG-WHO-AM-I-MASK_ --width=8


  /** EXPERIMENTAL */

  /**
  Returns the vector calculation: a magnitude and a heading in 3d space
  */
  read-acceleration-vector -> AccelOrientation:
    // Get new reading
    accel := read-acceleration

    // Magnitude (|accel| in g)
    magnitude/float := math.sqrt (accel.x * accel.x + accel.y * accel.y + accel.z * accel.z)

    // Pitch
    den-pitch/float := (math.sqrt (accel.y*accel.y + accel.z*accel.z)) + eps_
    pitch-radians/float := math.atan2 accel.x den-pitch
    pitch-degrees/float := pitch-radians * deg_

    // roll: rotation around X (left/right)
    den-roll/float := (math.sqrt (accel.x * accel.x + accel.z * accel.z)) + eps_
    roll-radians/float := math.atan2 accel.y den-roll
    roll-degrees/float := roll-radians * deg_

    // yaw is undefined from accel alone; keep null (or 0.0 if you prefer)
    return AccelOrientation magnitude pitch-degrees roll-degrees null





  /**
  Random Number Generation

  Following blog post: https://gist.github.com/bloc97/b55f684d17edd8f50df8e918cbc00f94

  Text: The MPU6050 is a multipurpose Accelerometer and Gyroscope sensor module
   for the Arduino, it can read raw acceleration from 3 axis and raw turn rate
   from 3 orientations. To our surprise, its acceleration sensor's noise level
   far surpasses its resolution, with at least 4 bits of recorded entropy.

  A naive approach to generate a random byte would to directly take the 4 least
   significant bits of the x and y axis, XORed with the z axis LSBs. //X, Y, Z
   are 4-bit values from each axis:

   randomByte := ((Y ^ Z) << 4) | (X ^ Z))

  Unfortunately this method is flawed as the distribution and bias of the noise
   is different and unpredictable between axes, not to mention other sensors of
   the same model. A simple fix would be to discard some bits and only use 2 bits
   from each axis, but that would yield only 6 bits of noise per reading, making
   it impossible to generate a 8-bit number with only one data sample of the
   accelerometer.

  However with clever transposition, we can achieve 8 bits of randomness using 4
   bits that are not necessarily the same magnitude from each axis. We are
   supposing that the upper 2 bits are not always reliable, so we will XOR each
   axis' higher 2 bits with another axis' lower 2 bits, and vice-versa.  An
   important property to note is the "piling-up lemma"[4], which states that
   XORing a good random source with a bad one is not harmful. Since we have 3
   axis, each having 4 bits, we will obtain 8 bits at the end. This operation
   is similar to Convolution:

   randomByte := ((X & 0x3) << 6) ^ (Z << 4) ^ (Y << 2) ^ X ^ (Z >> 2)

  This final method achieves state of the art performance for True Random Number
   Generation on the Arduino, with our tests providing us around 8000 random bits
   per second on an Arduino Uno.
  */
  get-random-number -> int:
    // Read acceleromter data
    a-x := read-register_ REG-ACCEL-XOUT_ --signed
    a-y := read-register_ REG-ACCEL-YOUT_ --signed
    a-z := read-register_ REG-ACCEL-ZOUT_ --signed

    // Use the second function described above to return a random byte
    random-byte := ((a-x & 0x3) << 6) ^ (a-z << 4) ^ (a-y << 2) ^ a-x ^ (a-z >> 2)
    return random-byte

  /**
  Reads and optionally masks/parses register data
  */
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

  /**
  Writes register data (masked or full register writes)
  */
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


  /** EXPERIMENTAL

  */

class AccelOrientation:
  magnitude/float ::= 0.0
  roll/float ::= 0.0
  pitch/float ::= 0.0
  yaw/float? ::= 0.0

  constructor .magnitude .roll .pitch .yaw:
