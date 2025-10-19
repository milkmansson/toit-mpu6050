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
  /**  Default $I2C-ADDRESS is 0x68  */
  static I2C-ADDRESS ::= 0x68
  static I2C-ADDRESS-AD0-1 ::= 0x69  // When pin AD0 is high
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
  static ACCEL-HPF-MASK_        ::= 0b00000111

  static ACCEL-HPF-RESET  ::= 0 // Reset
  static ACCEL-HPF-5HZ    ::= 1 // On @ 5 Hz
  static ACCEL-HPF-2-5HZ  ::= 2 // On @ 2.5 Hz
  static ACCEL-HPF-1-25HZ ::= 3 // On @ 1.25 Hz
  static ACCEL-HPF-0-63HZ ::= 4 // On @ 0.63 Hz
  static ACCEL-HPF-HOLD   ::= 7 // Hold

  static GYRO-FS-131-0 ::= 0  // 131 LSB/deg/s  +/- 250deg/s
  static GYRO-FS-65-5  ::= 1  // 65.5 LSB/deg/s  +/- 500deg/s
  static GYRO-FS-32-8  ::= 2  // 32.8 LSB/deg/s  +/- 1000deg/s
  static GYRO-FS-16-4  ::= 3  // 16.4 LSB/deg/s  +/- 2000deg/s

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

  // Cycle Wake-up Frequency
  static SLEEP-CYCLE-WAKE-FREQ-1-25HZ ::= 0  // 1.25 Hz
  static SLEEP-CYCLE-WAKE-FREQ-5HZ    ::= 1  // 5 Hz
  static SLEEP-CYCLE-WAKE-FREQ-20HZ   ::= 2  // 20 Hz
  static SLEEP-CYCLE-WAKE-FREQ-40HZ   ::= 3  // 40 Hz

  static CLOCK-SRC-INTERNAL-8MHZ    ::= 0 // Internal 8MHz oscillator
  static CLOCK-SRC-PLL-X-G          ::= 1 // PLL with X axis gyroscope reference
  static CLOCK-SRC-PLL-Y-G          ::= 2 // PLL with Y axis gyroscope reference
  static CLOCK-SRC-PLL-Z-G          ::= 3 // PLL with Z axis gyroscope reference
  static CLOCK-SRC-PLL-EXT-32768KHZ ::= 4 // PLL with external 32.768kHz reference
  static CLOCK-SRC-PLL-EXT-192MHZ   ::= 5 // PLL with external 19.2MHz reference
  static CLOCK-STOP                 ::= 7 // Stops the clock and keeps the timing generator in reset

  // Signal Path Reset
  static REG-SIGNAL-PATH-RESET_ ::= 0x68
  static SIGNAL-PATH-GYRO-RESET-MASK_  ::= 0b00000100
  static SIGNAL-PATH-ACCEL-RESET-MASK_ ::= 0b00000010
  static SIGNAL-PATH-TEMP-RESET-MASK_  ::= 0b00000001

  // Interrupts
  static REG-INTRPT-ENABLE_  ::= 0x38
  static REG-INTRPT-STATUS_  ::= 0x3a
  static INTRPT-FREEFALL-DETECT    ::= 0b1000_0000
  static INTRPT-MOTION-DETECT      ::= 0b0100_0000
  static INTRPT-ZERO-MOTION-DETECT ::= 0b0010_0000
  static INTRPT-FIFO-OVERFLOW      ::= 0b0001_0000
  static INTRPT-I2C-MASTER-SOURCES ::= 0b0000_1000
  static INTRPT-DMP                ::= 0b0000_0010
  static INTRPT-DATA-READY         ::= 0b0000_0001


  static REG-INTRPT-PIN-CONFIG_ ::= 0x37
  static INTRPT-PIN-ACTIVE-HL-MASK_       ::= 0b1000_0000  // 0 active high or 1 active low
  static INTRPT-PIN-PUSH-OPEN-MASK_       ::= 0b0100_0000  // 0 Push Pull or 1 Open Drain
  static INTRPT-PIN-LATCH-MASK_           ::= 0b0010_0000  // 0 50us pulse or 1 latch
  static INTRPT-READ-CLEAR-MASK_          ::= 0b0001_0000  // 0 manual clearing or 1 reads clear int
  static INTRPT-FSYNC-PIN-ACTIVE-HL-MASK_ ::= 0b0000_1000
  static INTRPT-FSYNC-PIN-ENABLE-MASK_    ::= 0b0000_0100
  static INTRPT-I2C-BYPASS-ENABLE-MASK_   ::= 0b0000_0010

  static REG-CONFIG_  ::= 0x1a
  static CONFIG-DLPF-CONFIG-MASK_  ::= 0b0000_0111
  static CONFIG-EXT-SYNC-SET-MASK_ ::= 0b0011_1000

  // Undocumented

  // Motion Detection
  static REG-MOT-DETECT-THRESHOLD_ ::= 0x1F
  static REG-MOT-DETECT-DURATION_  ::= 0x20
  static REG-MOT-DETECT-CTRL_      ::= 0x69

  // REG-MOT-DETECT-CTRL_ masks
  static MOT-DETECT-ACCEL-ON-DELAY-MASK_ ::= 0b0011_0000
  static MOT-DETECT-FF-COUNT-MASK_       ::= 0b0000_1100
  static MOT-DETECT-COUNT-MASK_          ::= 0b0000_0011

  // Zero Motion Detection
  static REG-ZMOT-DETECT-THRESHOLD_ ::= 0x21
  static REG-ZMOT-DETECT-DURATION_  ::= 0x22
  static ZMOT-LSB_ ::= 64

  //
  static REG-MOTION-DETECT-STATUS_ ::= 0x61
  static MOT-DETECT-X-NEG ::= 0b1000_0000
  static MOT-DETECT-X-POS ::= 0b0100_0000
  static MOT-DETECT-Y-NEG ::= 0b0010_0000
  static MOT-DETECT-Y-POS ::= 0b0001_0000
  static MOT-DETECT-Z-NEG ::= 0b0000_1000
  static MOT-DETECT-Z-POS ::= 0b0000_0100
  static MOT-DETECT-MOT-TO-ZMOT ::= 0b0000_0001

  //Enable the Zero-Motion interrupt in INT_ENABLE.

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
    logger_ = logger.with-name "mpu6050"
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
    reset-gyroscope
    reset-temperature
    reset-accelerometer
    enable-temperature
    set-clock-source CLOCK-SRC-INTERNAL-8MHZ // Internal 8MHz Oscillator
    set-accelerometer-full-scale ACCEL-FS-RANGE-8G      // Set accelerometer range to +/- 8g
    set-gyroscope-full-scale GYRO-FS-131-0

  /**
  Reading clears latching if manual
  */
  get-interrupt-status -> int:
    return read-register_ REG-INTRPT-STATUS_ --width=8

  /**
  All bits clear when read, except for the (Zero Motion to Motion) polarity
   indicator (bit 0) in mask $MOT-DETECT-MOT-TO-ZMOT
  */
  get-motion-detect-status -> int:
    return read-register_ REG-MOTION-DETECT-STATUS_ --width=8

  get-dlpf-config -> int:
    return read-register_ REG-CONFIG_ --mask=CONFIG-DLPF-CONFIG-MASK_ --width=8

  set-dlpf-config config/int -> none:
    assert: 0 <= config <= 7
    write-register_ REG-CONFIG_ config --mask=CONFIG-DLPF-CONFIG-MASK_  --width=8

  enable-motion-detection-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-MOTION-DETECT --width=8

  disable-motion-detection-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-MOTION-DETECT  --width=8

  enable-zero-motion-detection-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-ZERO-MOTION-DETECT --width=8

  disable-zero-motion-detection-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-ZERO-MOTION-DETECT  --width=8

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
  reset-gyroscope -> none:
    write-register_ REG-SIGNAL-PATH-RESET_ 1 --mask=SIGNAL-PATH-GYRO-RESET-MASK_ --width=8

  /**
  Resets Temperature Signal Path.
  */
  reset-temperature -> none:
    write-register_ REG-SIGNAL-PATH-RESET_ 1 --mask=SIGNAL-PATH-TEMP-RESET-MASK_ --width=8

  /**
  Resets Accelerometer Signal Path.
  */
  reset-accelerometer -> none:
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
  Enable "Accelerometer Only Low Power Mode".

  - Set CYCLE bit to 1
  - Set SLEEP bit to 0
  - Set TEMP_DIS bit to 1
  - Set STBY_XG, STBY_YG, STBY_ZG bits to 1

  Uses SLEEP-CYCLE-WAKE-FREQ-* Constants.
  */
  enable-accelerometer-only-low-power-mode setting/int -> none:
    assert: 0 <= setting <= 3
    enable-sleep-cycle setting
    wakeup-now
    disable-temperature
    disable-gyroscope-x
    disable-gyroscope-y
    disable-gyroscope-z

  /**
  Enable Sleep Cycle.

  When this is enabled (and SLEEP is disabled) the MPU-60X0 will cycle
   between sleep mode and waking up to take a single sample of data from active
   sensors, at a rate determined by LP_WAKE_CTRL (register 108).

  Uses SLEEP-CYCLE-WAKE-FREQ-* Constants.
  */
  enable-sleep-cycle setting/int -> none:
    assert: 0 <= setting <= 3
    write-register_ REG-POWER-MANAGEMENT_ 1 --mask=PM-CYCLE-MASK_ --width=16
    write-register_ REG-POWER-MANAGEMENT_ setting --mask=PM-LP-WAKE-CTRL-MASK_ --width=16

  /**
  Disable Sleep Cycle.

  When this is disabled (and SLEEP is disabled) the MPU-60X0 will not sleep in
  intervals.  See $enable-sleep-cycle
  */
  disable-sleep-cycle -> none:
    write-register_ REG-POWER-MANAGEMENT_ 0 --mask=PM-CYCLE-MASK_ --width=16

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

  Uses one of the CLOCK-* Constants
  */
  set-clock-source source/int=0 -> none:
    write-register_ REG-POWER-MANAGEMENT_ source --mask=PM-CLOCK-SOURCE-MASK_ --width=16


  /**
  Reads acceleration at this moment.

  Each 16-bit accelerometer measurement has a full scale defined in
  $ACCEL-FS-SELECT-MASK_ in register $REG-ACCEL-CONFIG_. For each full scale
  setting, the accelerometers’ sensitivity changes per LSB.
  */
  read-acceleration -> math.Point3f:
    a-fs := get-accelerometer-full-scale      // Obtain range configuration
    a-lsb := convert-accel-fs-to-value_ a-fs  // Obtain multiplier
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
  Set Accelerometer High Pass Filter

  3-bit unsigned value. Selects the Digital High Pass Filter configuration.
  ACCEL_HPF configures the DHPF available in the path leading to motion
  detectors (Free Fall, Motion threshold, and Zero Motion). The high pass filter
  output is not available to the data registers (see Figure in Section 8 of the
  MPU-6000/MPU-6050 Product Specification document).

  The high pass filter has three modes:
  - Reset: The filter output settles to zero within one sample.  This
    effectively disables the high pass filter. This mode may be toggled to
    quickly settle the filter.
  - On: The high pass filter will pass signals above the cut off frequency.
  - Hold: When triggered, the filter holds the present sample. The filter output
    will be the difference between the input sample and the held sample.
  */
  set-accelerometer-high-pass-filter raw/int -> none:
    assert: 0 <= raw <= 7
    write-register_ REG-ACCEL-CONFIG_ raw --mask=ACCEL-HPF-MASK_ --width=8

  /**
  Set Accelerometer High Pass Filter.  See $set-accelerometer-high-pass-filter
  */
  get-accelerometer-high-pass-filter -> int:
    return read-register_ REG-ACCEL-CONFIG_ --mask=ACCEL-HPF-MASK_ --width=8

  /**
  Gets Accelerometer scale value from the register.
  */
  get-accelerometer-full-scale -> int:
    return read-register_ REG-ACCEL-CONFIG_ --mask=ACCEL-FS-SELECT-MASK_ --width=8

  /**
  Sets Accelerometer scale values from the register.
  */
  set-accelerometer-full-scale raw/int -> none:
    assert: 0 <= raw <= 3
    write-register_ REG-ACCEL-CONFIG_ raw --mask=ACCEL-FS-SELECT-MASK_ --width=8

  /**
  Converts Accelerometer scale selectors to actual values/multipliers.
  */
  convert-accel-fs-to-value_ config/int -> int:
    // AFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= config <= 3
    if config == ACCEL-FS-RANGE-2G: return 16384 // ±2g 16384 LSB/g
    if config == ACCEL-FS-RANGE-4G: return 8192  // ±4g 8192 LSB/g
    if config == ACCEL-FS-RANGE-8G: return 4096  // ±8g 4096 LSB/g
    if config == ACCEL-FS-RANGE-16G: return 2048  // ±16g 2048 LSB/g
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
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-DATA-READY --width=8

  disable-data-ready-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-DATA-READY --width=8

  enable-i2c-master-sources-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-I2C-MASTER-SOURCES --width=8

  disable-i2c-master-sources-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-I2C-MASTER-SOURCES --width=8

  enable-fifo-overflow-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-FIFO-OVERFLOW --width=8

  disable-fifo-overflow-interrupt -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-FIFO-OVERFLOW --width=8

  set-interrupt-pin-active-high -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-PIN-ACTIVE-HL-MASK_ --width=8

  set-interrupt-pin-active-low -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-PIN-ACTIVE-HL-MASK_ --width=8

  enable-interrupt-pin-latching -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-PIN-LATCH-MASK_ --width=8

  disable-interrupt-pin-latching -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-PIN-LATCH-MASK_ --width=8

  set-interrupt-pin-push-pull -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-PIN-PUSH-OPEN-MASK_ --width=8

  set-interrupt-pin-open-drain -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-PIN-PUSH-OPEN-MASK_ --width=8

  set-interrupt-pin-manual-clear -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-READ-CLEAR-MASK_ --width=8

  set-interrupt-pin-read-clears -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-READ-CLEAR-MASK_ --width=8

  set-interrupt-fsync-pin-active-high -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-FSYNC-PIN-ACTIVE-HL-MASK_ --width=8

  set-interrupt-fsync-pin-active-low -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-FSYNC-PIN-ACTIVE-HL-MASK_ --width=8

  enable-fsync-pin -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-FSYNC-PIN-ENABLE-MASK_ --width=8

  disable-fsync-pin -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-FSYNC-PIN-ENABLE-MASK_ --width=8

  /**
  Enables I2C bypass.

  When this is enabled (and I2C_MST_EN (Register 106 bit[5]) is equal to 0) the
   host application processor will be able to directly access the auxiliary I2C
   bus of the MPU-60X0.
  */
  enable-i2c-bypass -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 1 --mask=INTRPT-I2C-BYPASS-ENABLE-MASK_ --width=8

  /**
  Disables I2C bypass.

  When this is not enabled, the host application processor will not be able to
  directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state
  of I2C_MST_EN (Register 106 bit[5]).
  */
  disable-i2c-bypass -> none:
    write-register_ REG-INTRPT-PIN-CONFIG_ 0 --mask=INTRPT-I2C-BYPASS-ENABLE-MASK_ --width=8

  /**
  Reads gyroscopic orientation at this moment.

  Each 16-bit gyroscope measurement has a full scale contained in
  $get-gyroscope-full-scale.  This obtained then converted to the LSB
  multiplier.
  */
  read-gyroscope -> math.Point3f:
    g-fs := get-gyroscope-full-scale          // Obtain range configuration
    g-lsb := convert-gyro-fs-to-value_ g-fs   // Obtain multiplier
    g-x := (read-register_ REG-GYRO-XOUT_ --signed) / g-lsb
    g-y := (read-register_ REG-GYRO-YOUT_ --signed) / g-lsb
    g-z := (read-register_ REG-GYRO-ZOUT_ --signed) / g-lsb
    return math.Point3f g-x g-y g-z

  /**
  Gets Gyroscope scale values from the register.
  */
  get-gyroscope-full-scale -> int:
    return read-register_ REG-GYRO-CONFIG_ --mask=GYRO-FS-SELECT-MASK_ --width=8

  /**
  Sets Gyroscope scale values from the register.
  */
  set-gyroscope-full-scale raw/int -> none:
    assert: 0 <= raw <= 3
    write-register_ REG-GYRO-CONFIG_ raw --mask=GYRO-FS-SELECT-MASK_ --width=8

  /**
  Converts Gyroscope scale selectors to actual values/multipliers.
  */
  convert-gyro-fs-to-value_ config/int -> float:
    // GFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= config <= 3
    if config == GYRO-FS-131-0: return 131.0 // ± 250  °/s 131 LSB/°/s
    if config == GYRO-FS-65-5:  return 65.5  // ± 500  °/s 65.5 LSB/°/s
    if config == GYRO-FS-32-8:  return 32.8  // ± 1000 °/s 32.8 LSB/°/s
    if config == GYRO-FS-16-4:  return 16.4  // ± 2000 °/s 16.4 LSB/°/s
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

  /** UNDOCUMENTED */

  /**
  Set Motion Detection Threshold.

  Motion detection threshold: higher = harder to trigger.
  */
  set-motion-detection-threshold-mg value/int -> none:
    assert: 0 <= value <= 255
    write-register_ REG-MOT-DETECT-THRESHOLD_ value --width=8

  /**
  Get configured Motion Detection Threshold.

  Motion detection threshold: higher = harder to trigger.
  */
  get-motion-detection-threshold-mg -> int:
    raw := read-register_ REG-MOT-DETECT-THRESHOLD_ --width=8
    return raw

  /**
  Set Motion Detection Duration.

  The motion counter runs at 1 kHz, so 1 LSB = 1 ms. A motion interrupt is
  asserted when the counter reaches this duration.
  */
  set-motion-detection-duration-ms value/int -> none:
    assert: 0 <= value <= 255
    write-register_ REG-MOT-DETECT-DURATION_ value --width=8

  /**
  Get configured Motion Detection Duration.

  The motion counter runs at 1 kHz, so 1 LSB = 1 ms. A motion interrupt is
  asserted when the counter reaches this duration.
  */
  get-motion-detection-duration-ms -> int:
    raw := read-register_ REG-MOT-DETECT-DURATION_ --width=8
    return raw


  /**
  Set Zero Motion Detection Duration.

  This register configures the duration counter threshold for Zero Motion
  interrupt generation. The duration counter ticks at 16 Hz, therefore the duration
  has a unit of 1 LSB = 64 ms. The Zero Motion duration counter increments while
  the absolute value of the accelerometer measurements are each less than the
  detection threshold. The Zero Motion interrupt is triggered when
  the Zero Motion duration counter reaches the time count specified in this
  register.
  */
  set-zero-motion-detection-duration-ms ms/int -> none:
    value/int := (ms / ZMOT-LSB_).to-int
    if (ms % ZMOT-LSB_) > 0:
      logger_.warn "set-zero-motion-detection-duration-ms: given $(ms)ms but LSB=$(ZMOT-LSB_) so rounding to $(value * ZMOT-LSB_)"
    value = clamp-value_ value --lower=0 --upper=255
    write-register_ REG-ZMOT-DETECT-DURATION_ value --width=8

  /**
  Get configured Zero Motion Detection Duration.

  See $set-zero-motion-detection-duration-ms
  */
  get-zero-motion-detection-duration-ms -> int:
    raw := read-register_ REG-ZMOT-DETECT-DURATION_ --width=8
    return raw * ZMOT-LSB_

  /**
  Set Zero Motion Detection Threshold.

  This register configures the detection threshold for Zero Motion interrupt
  generation. The mg per LSB increment for ZRMOT_THR can be found in the
  Electrical Specifications table of the MPU-6000/MPU-6050 Product Specification
  document.

  Zero Motion is detected when the absolute value of the accelerometer
  measurements for the 3 axes are each less than the detection threshold. This
  condition increments the Zero Motion duration counter. The Zero Motion
  interrupt is triggered when the Zero Motion duration counter reaches the time
  count specified in $get-zero-motion-detection-duration-ms

  Unlike Free Fall or Motion detection, Zero Motion detection triggers an
  interrupt both when Zero Motion is first detected and when Zero Motion is no
  longer detected.  When a zero motion event is detected, a Zero Motion Status
  will be indicated in the MOT_DETECT_STATUS register (Register 97). When a
  motion-to-zero-motion condition is detected, the status bit is set to 1. When
  a zero-motion-to-motion condition is detected, the status bit is set to 0.
  */
  set-zero-motion-detection-threshold-mg value/int -> none:
    value = clamp-value_ value --lower=0 --upper=255
    write-register_ REG-ZMOT-DETECT-THRESHOLD_ value --width=8

  /**
  Get configured Zero Motion Threshold.

  See $set-zero-motion-detection-duration-ms
  */
  get-zero-motion-detection-threshold-mg -> int:
    raw := read-register_ REG-ZMOT-DETECT-THRESHOLD_ --width=8
    return raw

  /**
  Set Acceleration Wake Delay.

  Extra acceleration wake delay: 4–7 ms (adds 0–3 ms beyond the default 4 ms).
  */
  set-acceleration-wake-delay-ms ms/int -> none:
    ms = clamp-value_ ms --lower=4 --upper=7
    value/int := ms - 4
    write-register_ REG-MOT-DETECT-CTRL_ value --mask=MOT-DETECT-ACCEL-ON-DELAY-MASK_ --width=8

  /**
  Get Acceleration Wake Delay.

  Extra acceleration wake delay: 0–3 ms (adds 0–3 ms beyond the default 4 ms).
  */
  get-acceleration-wake-delay-ms -> int:
    raw := read-register_ REG-MOT-DETECT-CTRL_ --mask=MOT-DETECT-ACCEL-ON-DELAY-MASK_ --width=8
    ms/int := raw + 4
    return ms

  /**
  Set Freefall counter decrement rate.

  Counter decrement rate: 0=reset to 0; 1=−1; 2=−2; 3=−4 per non-qualifying sample.
  */
  set-free-fall-count-decrement-rate value/int -> none:
    assert: 0 <= value <= 3
    write-register_ REG-MOT-DETECT-CTRL_ value --mask=MOT-DETECT-FF-COUNT-MASK_ --width=8

  /**
  Get Freefall counter decrement rate.

  Counter decrement rate: 0=reset to 0; 1=−1; 2=−2; 3=−4 per non-qualifying sample.
  */
  get-free-fall-count-decrement-rate -> int:
    raw := read-register_ REG-MOT-DETECT-CTRL_ --mask=MOT-DETECT-FF-COUNT-MASK_ --width=8
    return raw

  /**
  Set Motion Detection counter decrement rate.

  Counter decrement rate: 0=reset to 0; 1=−1; 2=−2; 3=−4 per non-qualifying sample.
  */
  set-motion-detection-count-decrement-rate value/int -> none:
    assert: 0 <= value <= 3
    write-register_ REG-MOT-DETECT-CTRL_ value --mask=MOT-DETECT-COUNT-MASK_ --width=8

  /**
  Get Motion Detection counter decrement rate.

  Counter decrement rate: 0=reset to 0; 1=−1; 2=−2; 3=−4 per non-qualifying sample.
  */
  get-motion-detection-count-decrement-rate -> int:
    raw := read-register_ REG-MOT-DETECT-CTRL_ --mask=MOT-DETECT-COUNT-MASK_ --width=8
    return raw

  /** EXPERIMENTAL */

  /**
  Returns the vector calculation: a magnitude and a heading in 3d space.

  Note that Yaw is currently fixed at null.
  */
  read-acceleration-vector -> AccelOrientation:
    // Get new reading
    accel := read-acceleration

    // Magnitude (|accel| in g)
    magnitude/float := magnitude_ accel

    // Pitch
    den-pitch/float := (math.sqrt (accel.y*accel.y + accel.z*accel.z)) + eps_
    pitch-radians/float := math.atan2 accel.x den-pitch
    pitch-degrees/float := pitch-radians * deg_

    // roll: rotation around X (left/right)
    den-roll/float := (math.sqrt (accel.x * accel.x + accel.z * accel.z)) + eps_
    roll-radians/float := math.atan2 accel.y den-roll
    roll-degrees/float := roll-radians * deg_

    // yaw is undefined from accel alone; keep null (or 0.0 if you prefer)
    return AccelOrientation magnitude roll-degrees pitch-degrees null

  magnitude_ obj/math.Point3f -> float:
    return math.sqrt (obj.x * obj.x + obj.y * obj.y + obj.z * obj.z)

  /**
  Whether the device is still or not when called.
  is-still --samples/int=20 --accel-g-eps/float=0.05 --gyro-dps-max/float=1.0 --sleep-ms/int=10 -> bool:
    //running-ema := ema.Ema
    count/int := 0
    samples.repeat:
      accel := read-acceleration
      gyro := read-gyroscope
      accel-magnitude := magnitude_ accel
      gyro-magnitude := magnitude_ gyro
      //running-ema.add accel-magnitude + gyro-magnitude

      if ((accel-magnitude - 1.0).abs <= accel-g-eps) and (gyro-magnitude <= gyro-dps-max):
        count += 1
      else:
        count = 0
      sleep --ms=sleep-ms
    return true
  */

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


/**
Class used to keep properties

Once created in degrees or radians, getters/setters stay in that method.
*/
class AccelOrientation:
  // range-max_/float ::= ?
  // range-min_/float ::= ?
  degrees_/bool ::= ?
  magnitude_/float? := null
  pitch_/float? := null
  roll_/float? := null
  yaw_/float? := null

  constructor .magnitude_ .pitch_ .roll_ .yaw_  --in-degrees=true --in-radians=false:
    assert: in-degrees != in-radians
    if in-radians:
      degrees_ = false
      // range-max_ = 2 * math.PI
      // range-min_ = 0.0
    else:
      degrees_ = true
      // range-max_ = 360.0
      // range-min_ = 0.0

  in-degrees -> bool:
    return degrees_

  in-radians -> bool:
    return not degrees_

  magnitude -> float?:
    return magnitude_

  magnitude= value/float? -> none:
    magnitude_ = value

  pitch -> float?:
    return pitch_

  roll -> float?:
    return roll_

  yaw -> float?:
    return yaw_

  /* For now, prevent changes
  pitch= value/float? -> none:
    assert: range-min_ <= value < range-max_
    pitch_ = value

  roll= value/float? -> none:
    assert: range-min_ <= value < range-max_
    roll_ = value

  yaw= value/float? -> none:
    assert: range-min_ <= value < range-max_
    yaw_ = value
  */

  to-string -> string:
    return "mag=$(%0.4f magnitude_) pitch=$(%0.3f pitch_) roll=$(%0.3f roll_) yaw=$(%0.3f yaw_) [$(degrees_ ? "(deg)" : "(rad)")]"
