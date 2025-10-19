# Toit driver for Invensense (TDK) MPU6000 and MPU6050 I2C Accelerometer and Gyroscope



![Front and back of an as5600](images/mpu6050.jpg)

> [!WARNING]
> This device is allegedly obsolete.  It is quoted as being noisy and outdated.
> However they are still cheap, widely available, and good enough for many
> projects such as mine, making the driver worth the time to write.

## Features

### Gyroscope Data
Each gyroscope sample from the MPU6050 gives angular velocity, i.e., how fast
the device is rotating around each axis at that moment.  its usually represented
in degrees/second for each of the three axes.  Note that this is not a universal
orientation, but a measure of **change** of orientation.
This 3-part measurement is obtained using Toit
[Math](https://libs.toit.io/math/library-summary) library's
[`Point3f`](https://libs.toit.io/math/class-Point3f) object.  Gyroscope
measurement can be obtained by:
```Toit
// Required for math objects
import math

// I2C setup omitted
new-gyro := ?
new-gyro = mpu6050-driver.read-gyroscope    // returns math.Point3f object

// Print single measurement
print "$(%0.2f new-gyro.x)deg/sec x"
print "$(%0.2f new-gyro.y)deg/sec y"
print "$(%0.2f new-gyro.z)deg/sec z"
```

### Accelerometer Data
One measurement from the accelerometer on the MPU6050 represents the
instantaneous acceleration being experienced by the sensor, split along its
three orthogonal sensing axes (X, Y, Z).  Similar to the gyroscope, the xyz data
is provided in a Toit [`Point3f`](https://libs.toit.io/math/class-Point3f)
object, as per this example:
```Toit
// Required for math objects
import math

// I2C setup omitted
new-accel := ?
new-accel = mpu6050-driver.read-acceleration    // returns math.Point3f object

// Print single measurement
print "$(%0.2f new-accel.x)g x"
print "$(%0.2f new-accel.y)g y"
print "$(%0.2f new-accel.z)g z"
```
See the 'continuous-accel-read' example.  Note that when the device is sitting
there doing nothing physical, the axis representing gravity will remain at the
value of g (gravitational acceleration - 9.80665 m/s²), for that axis:
```Text
[jaguar] INFO: program ea3248cd-18e0-7373-4ee4-0cff10ff4ebe started
 Found Mpu60x0 on 0x68
 get-whoami returned: 0x34
 get-temperature returned: 20.483c
 execute-gyro-self-test now:
 read-accel returned: 1.0459x.g 0.0337y.g 0.0269z.g
 read-accel returned: 1.0437x.g 0.0312y.g 0.0295z.g
 read-accel returned: 1.0488x.g 0.0312y.g 0.0288z.g
 read-accel returned: 1.0447x.g 0.0234y.g 0.0276z.g
 read-accel returned: 1.0388x.g 0.0386y.g 0.0247z.g
 read-accel returned: 1.0457x.g 0.0234y.g 0.0156z.g
 read-accel returned: 1.0457x.g 0.0356y.g 0.0273z.g
 ...
```

### Motion Detection
This device can provide alerts when going from zero motion into motion, and vice
versa.  There are thresholds determining how much motion is necessary to trigger
each of these.  Its a somewhat undocumented feature however it functions very
well using information from the community.

### Temperature gauge
The device has its own thermal sensor. Its range is good for –40C to +85C with
increments of 0.00294C, however the overall accuracy is specified at +/- 1C. Temperature can
be obtained with:
```Toit
// I2C setup omitted
print " get-temperature: $(%0.3f driver.read-temperature)c"  // e.g.  24.213c
```

### FIFO Buffer
MPU6050 has a 1024 byte FIFO buffer which allows reads to occur in bursts.  This
can be enabled using `enable-fifo-buffer` and `disable-fifo-buffer`.  If using
this mode,  :
```

```

## Undocumented Features
The MPU6050 has allegedy many undocumented registers that the community knows
and continues to use.  Functions that have been implemented based on these
undocumented features are:
- Freefall detection
- X/Y/Z Fine Gain Control
- DMP - Onboard 'Motion Processing'.  This was never fully documented but has
  been undergoing reverse engineering by the community, which is still
  incomplete.  A blob of several KB needs to be uploaded to the registers upon
  power up.  I'm not 100% sure of embedding a blob and providing it here...
  Instead for now I'll do my best for implementations using math on the ESP32
  CPU.
- Others...

### Sources
Links to sources of information about these:
- [Arduino Forums Post 'Reverse Engineering Undocumented MPU6050 Registers'](https://forum.arduino.cc/t/reverse-engineering-undocumented-mpu6050-registers/698986/2)
- [MPU6050 register list on I2CDevLib.com](https://www.i2cdevlib.com/devices/mpu6050#registers)

## Experimental Features
Whilst I expect these won't be useful to anyone, I've used the MPU6050 for a
couple of other features/outcomes/reasons in the past:

### Magnitude
By using the accelerometer output a magnitude vector can be established of the
total force (in whatever direction, as opposed to trying to track the size in three
dimensions simultaneously).
```Toit
// I2C setup omitted
mpu6050-driver.magnitude mpu6050-driver.read-acceleration

// eg
```

This value, combined with pitch roll and yaw is with some values indicating direction.  To account for
these values together there is a separate object, as a Toit-native 'set' cannot
contain floats.  Ensure you configure the range to suit your expected strength,
and note that yaw is undefined from acceleration alone - the value is kept at
null.
```Toit
// I2C setup omitted

// Set expected range (in g's) from 2, 4, 8 or 16, using the constants.
mpu6050-driver.set-accel-fs ACCEL-FS-RANGE-8G
object := mpu6050-driver.read-acceleration-vector

// Print single measurement
300.repeat:
  print "$(%0.2f object.magnitude)g"
  sleep --ms=100
```
Note that with very slow gentle movement, the magnitude value stays pretty close
to 1, in whatever direction that it comes to rest.  The other values (pitch/roll) change to show the movement direction in 3 dimensions (roll/pitch/yaw).

## Features not implemented yet
- MPU6050 as an auxiliary independent I2C bus, hosted on pins XDA/XCL.  Whilst
  this device is small/affordable, similar devices with magnetometers already
  built-in already exist and are easily available.
- Self-Test
- DMP - 'Digital Motion Processor' and 'Motion Fusion' features
- 1024 Byte FIFO buffer
- Full AHRS implementation. See: [Open Source IMU and AHRS algorithms](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/) and [Fusion Library](https://github.com/xioTechnologies/Fusion)

## Links
- [Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

## Issues
If there are any issues, changes, or any other kind of feedback, please
[raise an issue](https://github.com/milkmansson/toit-mpu60x0/issues). Feedback is
welcome and appreciated!

## Disclaimer
- This driver has been written and tested with an unbranded module as pictured.
- All trademarks belong to their respective owners.
- No warranties for this work, express or implied.

## Credits
- Credit to [imliubo](https://github.com/imliubo) for their Toit implementation
  for sibiling device [MPU6886](https://github.com/imliubo/mpu6886-toit).
- AI has been used for reviews, analysing & compiling data/results, and
  assisting with ensuring accuracy.
- [Florian](https://github.com/floitsch) for the tireless help and encouragement
- The wider Toit developer team (past and present) for a truly excellent product

## About Toit
One would assume you are here because you know what Toit is.  If you dont:
> Toit is a high-level, memory-safe language, with container/VM technology built
> specifically for microcontrollers (not a desktop language port). It gives fast
> iteration (live reloads over Wi-Fi in seconds), robust serviceability, and
> performance that’s far closer to C than typical scripting options on the
> ESP32. [[link](https://toitlang.org/)]
- [Review on Soracom](https://soracom.io/blog/internet-of-microcontrollers-made-easy-with-toit-x-soracom/)
- [Review on eeJournal](https://www.eejournal.com/article/its-time-to-get-toit)
