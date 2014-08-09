# IMU #
##### Alternative Code For AurduIMU v3 #####

This code makes heavy use of Martin Crown's code on the diydrones.com forum. His sketch is based on Jeff Rowberg’s [MPU6050 I2C library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050). The MPU6050 used in the original sketch however is the little brother of the MPU6000 in the Invensense lineup and only supports I2C. [Martin’s sketch](http://diydrones.com/forum/topics/find-here-working-arduino-sketch-for-mpu-6000-arduimu-v3-using) is a port of Jeff’s sketch that also uses the DMP, but takes advantage of the SPI port instead of I2C.

The on chip Digital Motion Processor (DMP) contained within the MPU6000 can generate 6-axis quaternion data, which is the fused data of the accelerometer and the gyroscope sensors. Roll, pitch and yaw angles can be extracted from the [quaternion](http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) data and this happens directly on the IMU processor. This alleviates some of the processing load from the flight controller.

I have since adapted Martin’s code adding/removing/modifying as necessary. The IMU communicates with the flight controller board via I2C, sending messages whenever new orientation data is available. 

### Teapot Demo ###
Martin’s code remains compatible with the [Teapot demo](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/Examples/MPU6050_DMP6) (a Processing visualization sketch) which can be very useful for testing/debugging visually. To run the Teapot demo on the PC, you’ll need to have [Processing](http://www.processing.org/) installed and copy an additional [toxiclibs library](http://toxiclibs.org/downloads/) into Processing’s workspace’s libraries folder. I've uploaded a video of the ArduIMU working with the Teapot Processing demo. 
http://www.benripley.com/diy/arduino/implementing-a-quadcopter-imu/ 

### Programming The Board ###
In order to program the ArduIMU v3 board, you need an [FTDI Cable](https://www.sparkfun.com/products/9716). An FTDI cable is a USB to Serial (TTL level) converter which allows for a simple way to connect TTL interface devices to USB.

### Relevant Links ###
[ArduIMU v3 Google Code Site](https://code.google.com/p/ardu-imu/)

[Invensense - MPU6000 Datasheet](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Components/General%20IC/PS-MPU-6000A.pdf)

[Martin Crown's MPU6000 Sketch](http://diydrones.com/forum/topics/find-here-working-arduino-sketch-for-mpu-6000-arduimu-v3-using)

[Jeff Rowberb's MP6050 Sketch](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)

[Alternative ArduIMU v3 Code](https://code.google.com/p/arduimu-v3-ahrs-improvements/)
