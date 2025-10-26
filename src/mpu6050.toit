// Copyright (C) 2025 Toit Contributors
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.   This file also includes derivative
// work from other authors and sources with permission.  See README.md.

import log
import math show *
import binary
import serial.device as serial
import serial.registers as registers
import io.buffer

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
  static ACCEL-FS-RANGE-2G  ::= 0  // 16384 LSB/g  +/- 2g
  static ACCEL-FS-RANGE-4G  ::= 1  // 8192 LSB/g  +/- 4g
  static ACCEL-FS-RANGE-8G  ::= 2  // 4096 LSB/g  +/- 8g
  static ACCEL-FS-RANGE-16G ::= 3  // 2048 LSB/g  +/- 16g

  static REG-GYRO-XOUT_ ::= 0x43
  static REG-GYRO-YOUT_ ::= 0x45
  static REG-GYRO-ZOUT_ ::= 0x47
  static REG-GYRO-CONFIG_ ::= 0x1b
  static GYRO-X-SELFTEST-MASK_ ::= 0b10000000
  static GYRO-Y-SELFTEST-MASK_ ::= 0b01000000
  static GYRO-Z-SELFTEST-MASK_ ::= 0b00100000
  static GYRO-FS-SELECT-MASK_ ::= 0b00011000
  static GYRO-FS-131-0 ::= 0  // 131 LSB/deg/s  +/- 250deg/s
  static GYRO-FS-65-5  ::= 1  // 65.5 LSB/deg/s  +/- 500deg/s
  static GYRO-FS-32-8  ::= 2  // 32.8 LSB/deg/s  +/- 1000deg/s
  static GYRO-FS-16-4  ::= 3  // 16.4 LSB/deg/s  +/- 2000deg/s

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
  static PM-LP-WAKE-CTRL-MASK_ ::= 0b00000000_10000000
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
  static INTRPT-FREEFALL-DETECT-MASK_    ::= 0b10000000
  static INTRPT-MOTION-DETECT-MASK_      ::= 0b01000000
  static INTRPT-ZERO-MOTION-DETECT-MASK_ ::= 0b00100000
  static INTRPT-FIFO-OVERFLOW-MASK_      ::= 0b00010000
  static INTRPT-I2C-MASTER-SOURCES-MASK_ ::= 0b00001000
  static INTRPT-PLL-READY-MASK_          ::= 0b00000100
  static INTRPT-DMP-MASK_                ::= 0b00000010
  static INTRPT-DATA-READY-MASK_         ::= 0b00000001

  static REG-INTRPT-PIN-CONFIG_ ::= 0x37
  static INTRPT-PIN-ACTIVE-HL-MASK_       ::= 0b10000000  // 0 active high or 1 active low
  static INTRPT-PIN-PUSH-OPEN-MASK_       ::= 0b01000000  // 0 Push Pull or 1 Open Drain
  static INTRPT-PIN-LATCH-MASK_           ::= 0b00100000  // 0 50us pulse or 1 latch
  static INTRPT-READ-CLEAR-MASK_          ::= 0b00010000  // 0 manual clearing or 1 reads clear int
  static INTRPT-FSYNC-PIN-ACTIVE-HL-MASK_ ::= 0b00001000
  static INTRPT-FSYNC-PIN-ENABLE-MASK_    ::= 0b00000100
  static INTRPT-I2C-BYPASS-ENABLE-MASK_   ::= 0b00000010

  static REG-CONFIG_  ::= 0x1a
  static CONFIG-EXT-SYNC-SET-MASK_ ::= 0b0011_1000
  static CONFIG-DLPF-CONFIG-MASK_  ::= 0b0000_0111
  static CONFIG-DLPF-0 ::= 0  // ACCEL= 260 Hz, 0 ms   / GYRO= 256 Hz, 0.98 ms / SR= 8 kHz
  static CONFIG-DLPF-1 ::= 1  // ACCEL= 184 Hz, 2.0 ms / GYRO= 188 Hz, 1.9 ms  / SR= 1 kHz
  static CONFIG-DLPF-2 ::= 2  // ACCEL= 94 Hz, 3.0 ms  / GYRO= 98 Hz, 2.8 1 ms / SR= 1 kHz
  static CONFIG-DLPF-3 ::= 3  // ACCEL= 44 Hz, 4.9 ms  / GYRO= 42 Hz, 4.8 1 ms / SR= 1 kHz
  static CONFIG-DLPF-4 ::= 4  // ACCEL= 21 Hz, 8.5 ms  / GYRO= 20 Hz, 8.3 1 ms / SR= 1 kHz
  static CONFIG-DLPF-5 ::= 5  // ACCEL= 10 Hz, 13.8 ms / GYRO= 10 Hz, 13.4 ms  / SR= 1 kHz
  static CONFIG-DLPF-6 ::= 6  // ACCEL= 5 Hz, 19.0 ms  / GYRO= 5 Hz, 18.6 1 ms / SR= 1 kHz

  // Feature/Capabilities Enabled
  static REG-USER-CTRL_          ::= 0x6A
  static USER-DMP-ENABLE-MASK_        ::= 0b10000000
  static USER-FIFO-ENABLE-MASK_       ::= 0b01000000
  static USER-I2C-MASTER-ENABLE-MASK_ ::= 0b00100000
  static USER-I2C-IF-DISABLE-MASK_    ::= 0b00010000
  static USER-DMP-RESET-MASK_         ::= 0b00001000  // Requires USER-DMP-ENABLE_ = 0
  static USER-FIFO-RESET-MASK_        ::= 0b00000100  // Requires USER-FIFO-ENABLE_ = 0
  static USER-I2C-MASTER-RESET-MASK_  ::= 0b00000010  // Requires USER-I2C-MASTER-ENABLE_ = 0
  static USER-SIG-COND-RESET-MASK_    ::= 0b00000001

  // FIFO Function Capability
  static REG-FIFO-COUNT_ ::= 0x72 // 16-bit
  static REG-FIFO-RW_ ::= 0x74
  static REG-FIFO-EN_ ::= 0x23
  static FIFO-EN-TEMP-MASK_    ::= 0b10000000
  static FIFO-EN-XG-MASK_      ::= 0b01000000
  static FIFO-EN-YG-MASK_      ::= 0b00100000
  static FIFO-EN-ZG-MASK_      ::= 0b00010000
  static FIFO-EN-ACCEL-MASK_   ::= 0b00001000
  static FIFO-EN-SLAVE2-MASK_  ::= 0b00000100  // EXT_SENS_DATA associated with Slave 2 to be written to the FIFO
  static FIFO-EN-SLAVE1-MASK_  ::= 0b00000010  // EXT_SENS_DATA associated with Slave 1 to be written to the FIFO
  static FIFO-EN-SLAVE0-MASK_  ::= 0b00000001  // EXT_SENS_DATA associated with Slave 0 to be written to the FIFO


  // Sampling Rate
  static REG-SMPLRT-DIVISOR_ ::= 0x19

  // Calibration (16 bit be's)
  static REG-ACCEL-X-USR-OFFSET_ ::= 0x06
  static REG-ACCEL-Y-USR-OFFSET_ ::= 0x08
  static REG-ACCEL-Z-USR-OFFSET_ ::= 0x0a
  static REG-GYRO-X-USR-OFFSET_ ::= 0x13
  static REG-GYRO-Y-USR-OFFSET_ ::= 0x15
  static REG-GYRO-Z-USR-OFFSET_ ::= 0x17

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
  static deg_/float ::= (180.0 / PI)
  static eps_/float ::= 1e-9

  // Globals
  reg_/registers.Registers := ?
  logger_/log.Logger := ?
  dmp-capable_/bool := false

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
    set-accelerometer-full-scale ACCEL-FS-RANGE-8G // Set accelerometer range to +/- 8g
    set-gyroscope-full-scale GYRO-FS-131-0


  /**
  This register specifies the divider from the gyroscope output rate used to
  generate the Sample Rate for the MPU-60X0. The sensor register output, FIFO
  output, DMP sampling, Motion detection, Zero Motion detection, and Free Fall
  detection are all based on the Sample Rate. The Sample Rate is generated by
  dividing the gyroscope output rate by SMPLRT_DIV: Sample Rate = Gyroscope
  Output Rate / (1 + SMPLRT_DIV) where Gyroscope Output Rate = 8kHz when the
  DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled (see
  Register 26). Note: The accelerometer output rate is 1kHz. This means that for
  a Sample Rate greater than 1kHz, the same accelerometer sample may be output
  to the FIFO, DMP, and sensor registers more than once. For a diagram of the
  gyroscope and accelerometer signal paths, see Section 8 of the MPU-
  6000/MPU-6050 Product Specification document.


  Remember to take care of this:
    if (st.chip_cfg.lp_accel_mode) {
        if (rate && (rate <= 40)) {
            /* Just stay in low-power accel mode. */
            mpu_lp_accel_mode(rate);
            return 0;
        }
        /* Requested rate exceeds the allowed frequencies in LP accel mode,
          * switch back to full-power mode.
        */
        mpu_lp_accel_mode(0);

  */
  set-sample-rate-hz hz/int -> none:
    if is-dmp-enabled:
      logger_.error "enable-interrupt-data-ready: Cannot set in DMP mode."
      return
    dlpf := get-dlpf-config
    base := 1000

    hz = clamp-value_ hz --lower=4 --upper=1000

    // dlpf = OFF adjust base if necessary:
    if (dlpf == 0) or (dlpf == 7): base = 8000

    divisor := (base / hz) - 1
    if divisor < 0: divisor = 0
    write-register_ REG-SMPLRT-DIVISOR_ divisor --width=8   // REG-SMPLRT-DIV

  get-sampling-rate-hz rate -> int:
    dlpf := get-dlpf-config
    base := 1000

    // dlpf = OFF adjust base if necessary:
    if (dlpf == 0) or (dlpf == 7): base = 8000
    divisor := read-register_ REG-SMPLRT-DIVISOR_ --width=8
    return base / (divisor + 1)

  /**
  Whether Digital Motion Processor is enabled.
  */
  is-dmp-enabled -> bool:
    raw := read-register_ Mpu6050.REG-USER-CTRL_ --mask=USER-DMP-ENABLE-MASK_ --width=8
    return raw == 1

  /**
  Sets Accelerometer User calibration values in their respective registers.
  */
  set-accelerometer-calibration input/Point3i -> none:
    if input.x != null:
      write-register_ REG-ACCEL-X-USR-OFFSET_ input.x --width=16 --signed
    if input.y != null:
      write-register_ REG-ACCEL-Y-USR-OFFSET_ input.y --width=16 --signed
    if input.z != null:
      write-register_ REG-ACCEL-Z-USR-OFFSET_ input.z --width=16 --signed

  /**
  Sets Accelerometer scale values from the register.
  */
  get-accelerometer-calibration -> Point3i:
    x := read-register_ REG-ACCEL-X-USR-OFFSET_ --width=16 --signed
    y := read-register_ REG-ACCEL-Y-USR-OFFSET_ --width=16 --signed
    z := read-register_ REG-ACCEL-Z-USR-OFFSET_ --width=16 --signed
    return Point3i x y z

  /**
  Sets Gyroscope User calibration values in their respective registers.
  */
  set-gyroscope-calibration input/Point3i -> none:
    if input.x != null:
      write-register_ REG-GYRO-X-USR-OFFSET_ input.x --width=16 --signed
    if input.y != null:
      write-register_ REG-GYRO-Y-USR-OFFSET_ input.y --width=16 --signed
    if input.z != null:
      write-register_ REG-GYRO-Z-USR-OFFSET_ input.z --width=16 --signed

  /**
  Sets Gyroscope scale values from the register.
  */
  get-gyroscope-calibration -> Point3i:
    x := read-register_ REG-GYRO-X-USR-OFFSET_ --width=16 --signed
    y := read-register_ REG-GYRO-Y-USR-OFFSET_ --width=16 --signed
    z := read-register_ REG-GYRO-Z-USR-OFFSET_ --width=16 --signed
    return Point3i x y z















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


  /**
  Gets Digital Low Pass Filtering Configuration.

  The DLPF_CFG parameter sets the digital low pass filter configuration. It also
   determines the internal sampling rate used by the device as shown in the table
   in README.md.

  Note: The accelerometer output rate is 1kHz. This means that for a Sample Rate
   greater than 1kHz, the same accelerometer sample may be output to the FIFO,
   DMP, and sensor registers more than once.
  */
  get-dlpf-config -> int:
    return read-register_ REG-CONFIG_ --mask=CONFIG-DLPF-CONFIG-MASK_ --width=8

  /**
  Sets Digital Low Pass Filtering Configuration - with DMP Guardrail.

  See $get-dlpf-config.
  */
  set-dlpf-config config/int -> none:
    assert: 0 <= config <= 7
    if is-dmp-enabled:
      logger_.error "set-dlpf-config: Cannot set while DMP enabled."
      return

    if config == CONFIG-DLPF-0:
      logger_.warn "set-dlpf-config: Accel sample rate fixed at 1khz.  With 8khz, Accel samples may show more than once."

    write-register_ Mpu6050.REG-CONFIG_ config --mask=Mpu6050.CONFIG-DLPF-CONFIG-MASK_  --width=8

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

  When DMP Driver is used, these work, and give low-level and fused data paths.
  However, When DMP is enabled, these reads are asynchronous — "off by a few"
  from what the DMP sees internally.
  */
  read-accelerometer --set/Point3i=(read-accelerometer-raw) -> Point3f:
    data/Point3i := ?
    fs := get-accelerometer-full-scale      // Obtain range configuration
    lsb := convert-accel-fs-to-value_ fs    // Obtain multiplier
    x := set.x.to-float / lsb
    y := set.y.to-float / lsb
    z := set.z.to-float / lsb
    return Point3f x y z

  /**
  Reads acceleration at this moment, returning raw register reads.
  */
  read-accelerometer-raw -> Point3i:
    x := read-register_ REG-ACCEL-XOUT_ --signed
    y := read-register_ REG-ACCEL-YOUT_ --signed
    z := read-register_ REG-ACCEL-ZOUT_ --signed
    return Point3i x y z

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
  Sets Accelerometer scale values from the register - with DMP guardrail.
  */
  set-accelerometer-full-scale raw/int -> none:
    assert: 0 <= raw <= 3
    if is-dmp-enabled:
      logger_.warn "set-accelerometer-full-scale: DMP Mode requires ACCEL-FS-RANGE-2G (0x00)" --tags={ "requested" : raw }
      write-register_ Mpu6050.REG-ACCEL-CONFIG_ Mpu6050.ACCEL-FS-RANGE-2G --mask=Mpu6050.ACCEL-FS-SELECT-MASK_ --width=8
    else:
      write-register_ Mpu6050.REG-ACCEL-CONFIG_ raw --mask=Mpu6050.ACCEL-FS-SELECT-MASK_ --width=8

  /**
  Converts Accelerometer scale selectors to actual values/multipliers.
  */
  convert-accel-fs-to-value_ config/int -> float:
    // AFS_SEL Full Scale Range LSB Sensitivity
    assert: 0 <= config <= 3
    if config == ACCEL-FS-RANGE-2G: return 16384.0 // ±2g 16384 LSB/g
    if config == ACCEL-FS-RANGE-4G: return 8192.0  // ±4g 8192 LSB/g
    if config == ACCEL-FS-RANGE-8G: return 4096.0  // ±8g 4096 LSB/g
    if config == ACCEL-FS-RANGE-16G: return 2048.0  // ±16g 2048 LSB/g
    return 0.0

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

  show-interrupts -> none:
    width := 8
    interrupts := read-register_ REG-INTRPT-ENABLE_ --width=width
    logger_.info "show-interrupts: $(bits-16_ interrupts --min-display-bits=width)"

  enable-interrupt-motion-detection -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-MOTION-DETECT-MASK_ --width=8

  disable-interrupt-motion-detection -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-MOTION-DETECT-MASK_  --width=8

  enable-interrupt-zero-motion-detection -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-ZERO-MOTION-DETECT-MASK_ --width=8

  disable-interrupt-zero-motion-detection -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-ZERO-MOTION-DETECT-MASK_  --width=8

  enable-interrupt-dmp -> none:
    logger_.error "enable-interrupt-dmp: DMP Mode not available in this driver." --tags={"driver":"mpu6050"}

  disable-interrupt-dmp -> none:
    logger_.error "enable-interrupt-dmp: DMP Mode not available in this driver." --tags={"driver":"mpu6050"}

  enable-interrupt-data-ready -> none:
    if is-dmp-enabled:
      logger_.error "enable-interrupt-data-ready: Cannot use in DMP mode."
      return
    else:
      write-register_ Mpu6050.REG-INTRPT-ENABLE_ 1 --mask=Mpu6050.INTRPT-DATA-READY-MASK_ --width=8

  disable-interrupt-data-ready -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-DATA-READY-MASK_ --width=8

  enable-interrupt-i2c-master-sources -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-I2C-MASTER-SOURCES-MASK_ --width=8

  disable-interrupt-i2c-master-sources -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-I2C-MASTER-SOURCES-MASK_ --width=8

  enable-interrupt-fifo-overflow -> none:
    write-register_ REG-INTRPT-ENABLE_ 1 --mask=INTRPT-FIFO-OVERFLOW-MASK_ --width=8

  disable-interrupt-fifo-overflow -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --mask=INTRPT-FIFO-OVERFLOW-MASK_ --width=8

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

  disable-all-interrupts -> none:
    write-register_ REG-INTRPT-ENABLE_ 0 --width=8

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
  read-gyroscope  --set/Point3i=(read-gyroscope-raw) -> Point3f:
    fs := get-gyroscope-full-scale          // Obtain range configuration
    lsb := convert-gyro-fs-to-value_ fs     // Obtain multiplier
    x := set.x.to-float / lsb
    y := set.y.to-float / lsb
    z := set.z.to-float / lsb
    return Point3f x y z

  /**
  Reads acceleration at this moment, returning raw register reads.
  */
  read-gyroscope-raw -> Point3i:
    x := read-register_ REG-GYRO-XOUT_ --signed
    y := read-register_ REG-GYRO-YOUT_ --signed
    z := read-register_ REG-GYRO-ZOUT_ --signed
    return Point3i x y z

  /**
  Gets Gyroscope scale values from the register.
  */
  get-gyroscope-full-scale -> int:
    return read-register_ REG-GYRO-CONFIG_ --mask=GYRO-FS-SELECT-MASK_ --width=8

  /**
  Sets Gyroscope scale values from the register - with DMP guardrail.
  */
  set-gyroscope-full-scale raw/int -> none:
    assert: 0 <= raw <= 3
    if is-dmp-enabled:
      logger_.warn "set-gyroscope-full-scale: DMP Mode requires GYRO-FS-16-4 (0x03)" --tags={ "requested" : raw }
      write-register_ REG-GYRO-CONFIG_ GYRO-FS-16-4 --mask=GYRO-FS-SELECT-MASK_ --width=8
    else:
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

















  /**
  Enables FIFO Buffer Capability.

  The FIFO buffer cannot be written to or read from while disabled. The FIFO
  buffer's state does not change unless the device is power cycled.  Reset is
  performed as part of enabling FIFO.
  */
  enable-fifo -> none:
    write-register_ REG-USER-CTRL_ 1 --mask=USER-FIFO-RESET-MASK_ --width=8
    write-register_ REG-USER-CTRL_ 1 --mask=USER-FIFO-ENABLE-MASK_ --width=8

  /**
  Disables FIFO Buffer Capability.

  The FIFO buffer cannot be written to or read from while disabled.  After being
  disabled, the FIFO buffer's state does not change unless the device is power
  cycled.
  */
  disable-fifo -> none:
    write-register_ REG-USER-CTRL_ 0 --mask=USER-FIFO-ENABLE-MASK_ --width=8
    // Unroute all sources from FIFO (disable all FIFO enabled sources)
    write-register_ REG-FIFO-EN_ 0 --width=8

  /**
  Clears the FIFO buffer.
  */
  clear-fifo -> none:
    sources := read-register_ REG-FIFO-EN_ --width=8
    enabled := read-register_ REG-USER-CTRL_ --mask=USER-FIFO-ENABLE-MASK_ --width=8
    write-register_ REG-FIFO-EN_ 0 --width=8
    write-register_ REG-USER-CTRL_ 1 --mask=USER-FIFO-RESET-MASK_ --width=8

    if enabled == 1:
      write-register_ REG-USER-CTRL_ enabled --mask=USER-FIFO-ENABLE-MASK_ --width=8
    if (read-register_ REG-FIFO-EN_ --width=8) != sources:
      write-register_ REG-FIFO-EN_ sources --width=8

    // Clear any existing interrupts
    nothing := read-register_ REG-INTRPT-STATUS_ --width=8

  /**
  Enables I2C Master Mode.

  When set to 1, this bit enables I2C Master Mode. When this bit is cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically driven by the primary I2C bus (SDA and SCL).
  */
  enable-i2c-master -> none:
    write-register_ REG-USER-CTRL_ 1 --mask=USER-I2C-MASTER-ENABLE-MASK_ --width=8

  /**
  Disables I2C Master Mode.

  When disabled, the auxiliary I2C bus lines (XDA and XCL) are logically
  driven by the primary I2C bus (SDA and SCL).
  */
  disable-i2c-master -> none:
    write-register_ REG-USER-CTRL_ 0 --mask=USER-I2C-MASTER-ENABLE-MASK_ --width=8

  /**
  Resets FIFO capability.

  Must be off to reset - reads previous state, disables and restores the
   previous state after reset.
  */
  reset-fifo -> none:
    enabled := read-register_ REG-FIFO-EN_ --mask=USER-FIFO-ENABLE-MASK_ --width=8
    write-register_ REG-USER-CTRL_ 0 --mask=USER-FIFO-ENABLE-MASK_ --width=8
    write-register_ REG-USER-CTRL_ 1 --mask=USER-FIFO-RESET-MASK_ --width=8
    sleep --ms=100
    if enabled == 1:
      write-register_ REG-USER-CTRL_ enabled --mask=USER-FIFO-ENABLE-MASK_ --width=8

  /**
  Resets I2C Master capability.

  Must be off to reset - reads previous state, disables and restores the
   previous state after reset.
  */
  reset-i2c-master -> none:
    enabled := read-register_ REG-USER-CTRL_ --mask=USER-I2C-MASTER-ENABLE-MASK_ --width=8
    write-register_ REG-USER-CTRL_ 0 --mask=USER-I2C-MASTER-ENABLE-MASK_ --width=8
    write-register_ REG-USER-CTRL_ 1 --mask=USER-I2C-MASTER-RESET-MASK_ --width=8
    sleep --ms=100
    if enabled == 1:
      write-register_ REG-USER-CTRL_ enabled --mask=USER-I2C-MASTER-ENABLE-MASK_ --width=8

  /**
  Resets Signal Paths.

  Resets the signal paths for all sensors (gyroscopes, accelerometers, and
   temperature sensor). This operation will also clear the sensor registers.
   When resetting only the signal path (and not the sensor registers), please
   use Register 104, SIGNAL_PATH_RESET.
  */
  reset-signal-paths -> none:
    write-register_ REG-USER-CTRL_ 1 --mask=USER-SIG-COND-RESET-MASK_ --width=8

  /**
  Gives the number of bytes stored in the FIFO buffer.

  This number is in turn the number of bytes that can be read from the FIFO
  buffer.  It is directly proportional to the number of samples available given
  the set of sensor data bound to be stored in the FIFO (register 35 and 36).
  */
  get-fifo-count -> int:
    raw := read-register_ REG-FIFO-COUNT_ --width=16
    return raw

  /**
  Collects all available bytes in the byte array and adds them all to the end of
   the supplied deque for user processing.
  */
  fifo-read deque/Deque -> Deque:
    bytes-to-read := get-fifo-count
    deque.reserve (deque.size + bytes-to-read)      // Reserve to reduce reallocs
    //while byte-buffer.is-empty: // may never end
    bytes-to-read.repeat:
      deque.add (read-register_ REG-FIFO-RW_ --width=8)
    return deque

  /**
  Collects and returns fifo queue, iff fifo queue data is accelerometer data.

  Interprets, maths out and returns a Deque of Point3f's (accelerometer 3tuples
   converted from i16-be, and into accelerometer data via LSBs etc.)
  */
  read-accelerometer-fifo --max-samples/int=32 -> Deque:
    fifo-channels-enabled := read-register_ REG-FIFO-EN_ --width=8
    if fifo-channels-enabled != FIFO-EN-ACCEL-MASK_:
      logger_.error "fifo-read-accel: can't run with wrong channels." --tags={"channels":fifo-channels-enabled}
      return Deque
    out := Deque

    // Get Data
    fifo-data := Deque
    fifo-data = fifo-read fifo-data

    tuples/int := (fifo-data.size / 6)    // floor
    if tuples > max-samples: tuples = max-samples

    tuples.repeat:
      x := i16-be_ fifo-data.remove-first fifo-data.remove-first --signed=true
      y := i16-be_ fifo-data.remove-first fifo-data.remove-first --signed=true
      z := i16-be_ fifo-data.remove-first fifo-data.remove-first --signed=true
      raw-accel := Point3i x y z
      accel := read-accelerometer --set=raw-accel
      out.add accel
    return out



































  /** EXPERIMENTAL */

  /**
  Returns the vector calculation: a magnitude and a heading in 3d space.

  Note that Yaw is currently fixed at null.
  */
  read-acceleration-vector -> Vector4f:
    // Get new reading
    accel := read-accelerometer

    // Magnitude (|accel| in g)
    magnitude/float := magnitude accel

    // Pitch
    den-pitch/float := (sqrt (accel.y*accel.y + accel.z*accel.z)) + eps_
    pitch-radians/float := atan2 accel.x den-pitch
    pitch-degrees/float := pitch-radians * deg_

    // roll: rotation around X (left/right)
    den-roll/float := (sqrt (accel.x * accel.x + accel.z * accel.z)) + eps_
    roll-radians/float := atan2 accel.y den-roll
    roll-degrees/float := roll-radians * deg_

    // yaw is undefined from accel alone; keep null (or 0.0 if you prefer)
    return Vector4f magnitude roll-degrees pitch-degrees null

  magnitude obj/Point3f -> float:
    return sqrt (obj.x * obj.x + obj.y * obj.y + obj.z * obj.z)

  normalize vector/Point3f -> Point3f:
    mag := magnitude vector
    if mag <= 1e-9: return Point3f 0.0 0.0 0.0
    inv := 1.0 / mag
    return Point3f (vector.x * inv) (vector.y * inv) (vector.z * inv)


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
    if (upper != null) and (lower != null):
      assert: upper > lower
    if upper != null: if value > upper:  return upper
    if lower != null: if value < lower:  return lower
    return value

  /**
  Parse big-endian 16bit from two separate bytes - eg when reading from FIFO.
  */
  i16-be_ high-byte/int low-byte/int --signed/bool=false -> int:
    high := high-byte & 0xFF
    low := low-byte & 0xFF
    value := (high << 8) | low
    if signed:
      return (value >= 0x8000) ? (value - 0x10000) : value
    else:
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
  Parse big-endian 32bit from byte array  - eg when reading from a packet.
  */
  i32-be_ packet/ByteArray --start/int --signed/bool=false -> int:
    if (start < 0) or (start + 3) >= packet.size:
      logger_.error "i32-be_: Out of bounds"
      throw "i32-be_: out of bounds."

    // Mask each byte just in case
    byte-0 := packet[start]      & 0xFF
    byte-1 := packet[start + 1]  & 0xFF
    byte-2 := packet[start + 2]  & 0xFF
    byte-3 := packet[start + 3]  & 0xFF

    value := (byte-0 << 24) | (byte-1 << 16) | (byte-2 << 8) | byte-3

    if signed and ((value & 0x80000000) != 0):
      return value - 0x1_0000_0000
    else:
      return value


/*

// Stores the reference “down” (gravity) direction in body frame and thresholds.
class OrientationProfile:
  // Saved unit vector of gravity in the neutral pose (points "down" in body frame)
  down0_/math.Point3f ::= math.Point3f 0.0 0.0 1.0

  // Thresholds in degrees, with hysteresis
  tilt-up-thresh-deg_/float ::= 12.0     // tilt toward down0 more than this => UP
  tilt-down-thresh-deg_/float ::= 12.0   // tilt away from down0 more than this => DOWN
  hysteresis-deg_/float ::= 3.0          // prevents flicker around thresholds

  // Internal latched state for hysteresis
  last_/TiltIntent ::= TiltIntent.NEUTRAL

  // Low-pass filter to estimate gravity
  lpf_/GravityLPF := GravityLPF --alpha=0.15

  // --- Calibration: call once while device is held still in the neutral pose ---
  calibrate read-accel/func()->math.Point3f --samples/int=100 --sleep-ms/int=5 -> none:
    sum := math.Point3f 0.0 0.0 0.0
    i := 0
    while i < samples:
      a := read-accel()         // should return accel in g-units
      sum = math.Point3f (sum.x + a.x) (sum.y + a.y) (sum.z + a.z)
      sleep --ms=sleep-ms
      i += 1
    avg := math.Point3f (sum.x / samples) (sum.y / samples) (sum.z / samples)
    down0_ = normalize avg      // store "down" vector
    lpf_.reset avg

  // --- Evaluate current intent: UP / DOWN / NEUTRAL ---
  evaluate read-accel/func()->math.Point3f -> TiltIntent:
    a := read-accel()
    g := lpf_.update a
    if (g.x == 0.0 and g.y == 0.0 and g.z == 0.0): return last_

    curDown := normalize g
    // angle between current down and saved down (degrees)
    cosTheta := curDown * down0_
    cosTheta = (cosTheta > 1.0) ? 1.0 : ((cosTheta < -1.0) ? -1.0 : cosTheta)
    theta := math.acos cosTheta
    thetaDeg := theta * (180.0 / math.PI)

    // “Toward down0” = small angle; “away” = large angle.
    upEnter  := (thetaDeg <= (tilt-up-thresh-deg_ - hysteresis-deg_))
    upExit   := (thetaDeg <= (tilt-up-thresh-deg_ + hysteresis-deg_))
    downEnter:= (thetaDeg >= (tilt-down-thresh-deg_ + hysteresis-deg_))
    downExit := (thetaDeg >= (tilt-down-thresh-deg_ - hysteresis-deg_))

    // 3-state latch with hysteresis
    if last_ == TiltIntent.UP:
      if upExit: return TiltIntent.UP else: last_ = TiltIntent.NEUTRAL
    if last_ == TiltIntent.DOWN:
      if downExit: return TiltIntent.DOWN else: last_ = TiltIntent.NEUTRAL

    // Neutral: check entries
    if upEnter:
      last_ = TiltIntent.UP
    else if downEnter:
      last_ = TiltIntent.DOWN
    else:
      last_ = TiltIntent.NEUTRAL

    return last_

  // Optional: let callers tweak thresholds/hysteresis at runtime
  set-thresholds --up-deg/float --down-deg/float --hyst-deg/float -> none:
    if up-deg != null:   tilt-up-thresh-deg_ = up-deg
    if down-deg != null: tilt-down-thresh-deg_ = down-deg
    if hyst-deg != null: hysteresis-deg_      = hyst-deg




// Simple exponential low-pass for accel -> gravity estimate.
class GravityLPF:
  g_/math.Point3f := math.Point3f 0.0 0.0 0.0
  alpha_/float := 0.1   // 0..1; higher = less smoothing

  constructor --alpha/float=0.1:
    alpha_ = alpha

  reset vector/math.Point3f -> none:
    g_ = vector

  update vector/math.Point3f -> math.Point3f:
    x := alpha_ * vector.x + (1 - alpha_) * g_.x
    y := alpha_ * vector.y + (1 - alpha_) * g_.y
    z := alpha_ * vector.z + (1 - alpha_) * g_.z
    g_ = math.Point3f x y z
    return g_

class TiltIntent:
//enum TiltIntent { DOWN, NEUTRAL, UP }


*/

/**
Small class to hold int versions of Point3f around
*/
class Point3i:
  x/int? := null
  y/int? := null
  z/int? := null

  constructor .x/int? .y/int? .z/int?:

/**
Small class to hold quaternion floats
*/

class Vector4i:
  w/int? := null
  x/int? := null
  y/int? := null
  z/int? := null

  constructor .w/int? .x/int? .y/int? .z/int?:

class Vector4f:
  w/float? := null
  x/float? := null
  y/float? := null
  z/float? := null

  constructor .w/float? .x/float? .y/float? .z/float?:

  getConjugate -> Vector4f:
    return Vector4f w -x -y -z

  magnitude -> float:
    return sqrt (w*w + x*x + y*y + z*z)

  normalize -> none:
    mag := magnitude
    w /= mag
    x /= mag
    y /= mag
    z /= mag
    return

  operator * q/Vector4f -> Vector4f:
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    new-w := w*q.w - x*q.x - y*q.y - z*q.z  // new w
    new-x := w*q.x + x*q.w + y*q.z - z*q.y  // new x
    new-y := w*q.y - x*q.z + y*q.w + z*q.x  // new y
    new-z := w*q.z + x*q.y - y*q.x + z*q.w  // new z
    return Vector4f new-w new-x new-y new-z
