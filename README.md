# Toit driver for TDK MPU6000 and MPU6050 I2C Accelerometer and Gyroscope



![Front and back of an as5600](images/mpu6050.jpg)


## Features

### Temperature gauge
The device has its own thermal sensor. Its range is good for –40C → +85C with
increments of 0.00294C, however the overall accuracy is +/-1C. Temperature can
be obtained with:
```Toit
// I2C setup omitted
print " get-temperature: $(%0.3f driver.read-temperature)c"  // e.g.  24.21c
```

### FIFO Buffer
MPU6050 has a 1024 byte FIFO buffer which allows reads to occur in bursts.  This
can be enabled using `enable-fifo-buffer` and `disable-fifo-buffer`.  If using
this mode,  :
```

```


### Features not implemented yet
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
