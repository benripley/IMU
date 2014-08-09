/*
# IMU - ArduIMU v3 #
This is code for the ArduIMU v3

This code makes heavy use of Martin Crown's code on diydrones.com. His 
sketch is based on Jeff Rowberg’s MPU6050_DMP6_I2C library. The MPU6050 
used in the original sketch however is the little brother of the MPU6000 
in the Invensense lineup and only supports I2C. Martin’s sketch is a port 
of Jeff’s sketch that also uses the DMP, but takes advantage of the SPI 
port instead of I2C.

The on chip Digital Motion Processor (DMP) contained within the MPU6000 
can generate 6-axis quaternion data, which is the fused data of the 
accelerometer and the gyroscope sensors. Roll, pitch and yaw angles 
can be extracted from the quaternion data and this happens directly on 
the IMU processor. This alleviates some of the processing load from 
the flight controller.

I have since adapted Martin’s code adding/removing/modifying as 
necessary. The IMU communicates with the flight controller board via 
I2C, sending messages whenever new orientation data is available. 

Teapot Demo 
Martin’s code remains compatible with the Teapot demo (a processing 
visualization sketch) which can be very useful for testing/debugging 
visually. To run the Teapot demo on the PC, you’ll need to have 
Processing installed and copy an additional toxiclibs library into 
Processing’s workspace’s libraries folder. I've uploaded a video of 
the ArduIMU working with the Teapot Processing demo.
http://www.benripley.com/diy/arduino/implementing-a-quadcopter-imu/ 

Programming The Board 
In order to program the ArduIMU v3 board, you need an FTDI Cable. An FTDI cable is a USB to Serial (TTL level) converter which allows for a simple way to connect TTL interface devices to USB.
https://www.sparkfun.com/products/9716

Relevant Links 
ArduIMU v3 Google Code Site: https://code.google.com/p/ardu-imu/
Invensense - MPU6000 Datasheet: http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Components/General%20IC/PS-MPU-6000A.pdf
Martin Crown's MPU6000 Sketch: http://diydrones.com/forum/topics/find-here-working-arduino-sketch-for-mpu-6000-arduimu-v3-using 
Jeff Rowberb's MP6050 Sketch: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 
Alternative ArduIMU v3 Code:https://code.google.com/p/arduimu-v3-ahrs-improvements/
*/


#include <SPI.h>  // http://arduino.cc/en/Reference/SPI
#include <Wire.h> // http://arduino.cc/en/reference/wire
#include <math.h> // http://www.arduino.cc/en/Math/H

#define RED_LED_PIN     5 // connected to red    LED on ArduIMU+ V3 (Z on the board)
#define BLUE_LED_PIN    6 // connected to blue   LED on ArduIMU+ V3 (Y on the board)
// NOTE: the yellow LED on ArduIMU+ V3 (X on the board) can not be set
// by the user because it is hardwired to the SPI clock (SCK)

// On the ArduIMU+ V3, output pin D7 on the ATmega328P controls a MUX which selects Serial Receive (RX)
// from a connected GPS module or from the FTDI (or similar) USB interface (more info further on).
#define SERIAL_MUX_PIN  7

// controls (red) LED on or off and at what blink speed
byte blinkState = 0x00, blink_divider = 0; 
const byte ADDR_SLAVE_I2C = 2;
const byte PACKET_SIZE = 12;

// On the ArduIMU+ V3, output pin D4 on the ATmega328P connects to
// input pin /CS (/Chip Select) of the MPU-6000. Without a correct definition
// here, there is no SPI data transfer possible and this sketch will not work.
const int ChipSelPin1 = 4; 

boolean dmpReady = false;     // set true if DMP initialization was successful
unsigned int packetSize = 42; // number of unique bytes of data written by the DMP each time (FIFO can hold multiples of 42-bytes)
unsigned int fifoCount;       // count of all bytes currently in FIFO
byte fifoBuffer[64];          // FIFO storage buffer (in fact only 42 used...)

// INTERRUPT FROM MPU-6000 DETECTION ROUTINE
volatile boolean mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}



// ############################################################################################## //
// ################################ SETUP ####################################################### //
// ############################################################################################## //
void setup()
  {
  pinMode(SERIAL_MUX_PIN, OUTPUT);   // Serial Mux Pin - SWITCHES SERIAL INPUT FROM GPS OR FTDI CHIP
  pinMode(RED_LED_PIN, OUTPUT);      // Red LED
  pinMode(BLUE_LED_PIN, OUTPUT);     // Blue LED

  Wire.begin(ADDR_SLAVE_I2C); // join I2C bus with Address #2
  Wire.onRequest(requestEvent); // register event
  
  Serial.begin(115200);

  // When LOW , set MUX to Receive Serial data from FTDI (upload sketch)
  // When HIGH, set MUX to Receive Serial data from GPS
  digitalWrite(SERIAL_MUX_PIN, LOW); 

  // Flush serial buffer to clean up remnants from previous run
  while(Serial.available() > 0) Serial.read(); 
  Serial.println();
  Serial.println("############# MPU-6000 Data Acquisition #############");
    
  //--- SPI settings ---//
  Serial.println("Initializing SPI Protocol...");
  SPI.begin();
  // ArduIMU+ V3 board runs on 16 MHz: 16 MHz / SPI_CLOCK_DIV16 = 1 MHz
  // 1 MHz is the maximum SPI clock frequency according to the MPU-6000 Product Specification
  SPI.setClockDivider(SPI_CLOCK_DIV16); 
  SPI.setBitOrder(MSBFIRST);  // data delivered MSB first as in MPU-6000 Product Specification
  SPI.setDataMode(SPI_MODE0); // latched on rising edge, transitioned on falling edge, active low
  Serial.println("...SPI Protocol initializing done.");
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
  delay(100);
  
  pinMode(ChipSelPin1, OUTPUT);
    
  // write & verify dmpMemory, dmpConfig and dmpUpdates into the DMP, and make all kinds of other settings
  Serial.println("Initializing Digital Motion Processor (DMP)...");
  byte devStatus; // return status after each device operation (0 = success, !0 = error)
  devStatus = dmpInitialize();

  // make sure it worked: dmpInitialize() returns a 0 in devStatus if so
  if (devStatus == 0)
  {
    // now that it's ready, turn on the DMP 
    Serial.print("Enabling DMP... ");
    SPIwriteBit(0x6A, 7, true, ChipSelPin1); // USER_CTRL_DMP_EN
    Serial.println("done.");

    // enable Arduino interrupt detection, this will execute dmpDataReady whenever there is an interrupt,
    // independing on what this sketch is doing at that moment
    // http://arduino.cc/en/Reference/AttachInterrupt
    Serial.print("Enabling interrupt detection... ");
    // attachInterrupt(interrupt, function, mode) specifies a function to call when an external interrupt occurs
    // ArduIMU+ V3 has ATMEGA328 INT0 / D2 pin 32 (input) connected to MPU-6000 INT pin 12 (output)
    attachInterrupt(0, dmpDataReady, RISING); // the 0 points correctly to INT0 / D2
    // -> if there is an interrupt from MPU-6000 to ATMEGA328, boolean mpuInterrupt will be made true
    byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared
    Serial.println("done.");

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    Serial.println("DMP ready! Waiting for first data from MPU-6000...");
    Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    
  }
  else
  {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

// ############################################################################################## //
// ################################ MAIN LOOP ################################################### //
// ############################################################################################## //
// Sensor orientation ArduIMU+ V3:
// (all rotations "through" the MPU-6000 on the board)
// +X is ROLL   rotation along longer dimension of the board (towards GPS connector - "front")
// +Y is PITCH  rotation along shorter dimension of the board (towards GPS connector - "left")
// +Z is YAW    rotation around line upwards (through the MPU-6000 - "top")

void loop()
{
  // if DMP initialization during setup failed, don't try to do anything
  if (!dmpReady) { return; }

  // wait for MPU interrupt or extra packet(s) available
  // INFO: if there is an interrupt send from the MPU-6000 to the ATmega328P (the "Arduino" processor),
  // boolean variable "mpuInterrupt" will be made "true" (see explanation in void setup() )
  while ((mpuInterrupt == false) && (fifoCount < packetSize)) { }

  // there has just been an interrupt, so reset the interrupt flag, then get INT_STATUS byte
  mpuInterrupt = false;
  byte mpuIntStatus = SPIread(0x3A, ChipSelPin1); // by reading INT_STATUS register, all interrupts are cleared

  // get current FIFO count
  fifoCount = getFIFOCount(ChipSelPin1);
  
  // check for FIFO overflow (this should never happen unless our code is too inefficient or DEBUG output delays code too much)
  // mpuIntStatus & 0x10 checks register 0x3A for FIFO_OFLOW_INT
  // the size of the FIFO buffer is 1024 bytes, but max. set to 1008 so after 24 packets of 42 bytes
  // the FIFO is reset, otherwise the last FIFO reading before reaching 1024 contains only 16 bytes
  // and can/will produces output value miscalculations 
  if ((mpuIntStatus & 0x10) || fifoCount == 1008)
  {
    // reset so we can continue cleanly
    //SPIwriteBit(0x6A, 6, false, ChipSelPin1); // FIFO_EN = 0 = disable
    SPIwriteBit(0x6A, 2, true, ChipSelPin1); // FIFO_RESET = 1 = reset (ok) only when FIFO_EN = 0
    //SPIwriteBit(0x6A, 6, true, ChipSelPin1); // FIFO_EN = 1 = enable

    digitalWrite(BLUE_LED_PIN, HIGH); // shows FIFO overflow without disturbing output with message
  }
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  // mpuIntStatus & 0x02 checks register 0x3A for (undocumented) DMP_INT
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = getFIFOCount(ChipSelPin1); 
    digitalWrite(BLUE_LED_PIN, LOW); // LED off again now that FIFO overflow is resolved

    // read a packet from FIFO
    SPIreadBytes(0x74, packetSize, fifoBuffer, ChipSelPin1);

    // track FIFO count here in case there is more than one packet (each of 42 bytes) available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount = fifoCount - packetSize;
  }

  // blink LED to indicate activity
  blink_divider ++;
  if (blink_divider == 10) // toggle LED on/off every 10 main loops, otherwise it blinks so fast it looks like always on
  {
    blinkState = ~blinkState; // byte toggling between FF and 00
    if (blinkState == 0x00) digitalWrite(RED_LED_PIN, LOW);
    if (blinkState == 0xFF) digitalWrite(RED_LED_PIN, HIGH);
    blink_divider = 0;
  }

} // end loop()





// runs whenever I2C data is requested by master
void requestEvent()
{
  // get the quaternion values from the FIFO 
  int raw_q_w = ((fifoBuffer[0] << 8)  + fifoBuffer[1]);  // W
  int raw_q_x = ((fifoBuffer[4] << 8)  + fifoBuffer[5]);  // X
  int raw_q_y = ((fifoBuffer[8] << 8)  + fifoBuffer[9]);  // Y
  int raw_q_z = ((fifoBuffer[12] << 8) + fifoBuffer[13]); // Z
  float q_w = raw_q_w / 16384.0f;
  float q_x = raw_q_x / 16384.0f;
  float q_y = raw_q_y / 16384.0f;
  float q_z = raw_q_z / 16384.0f;
  
  // Output Euler Angles
  float euler_x = atan2((2 * q_y * q_z) - (2 * q_w * q_x), (2 * q_w * q_w) + (2 * q_z * q_z) - 1); // phi
  float euler_y = -asin((2 * q_x * q_z) + (2 * q_w * q_y));                                        // theta
//  float euler_z = atan2((2 * q_x * q_y) - (2 * q_w * q_z), (2 * q_w * q_w) + (2 * q_x * q_x) - 1); // psi
  euler_x = euler_x * 180/M_PI; // angle in degrees -180 to +180
  euler_y = euler_y * 180/M_PI; // angle in degrees -180 to +180
//  euler_z = euler_z * 180/M_PI; // angle in degrees -180 to +180
  volatile byte* Rol_Ptr = (byte*) &euler_x;
  volatile byte* Pit_Ptr = (byte*) &euler_y;
//  volatile byte* Yaw_Ptr = (byte*) &euler_z;

// print gyroscope values from fifoBuffer
float GyroZ = ((fifoBuffer[24] << 8) + fifoBuffer[25]);
volatile byte* Yaw_Ptr = (byte*) &GyroZ;

  byte* output_packet;
  output_packet[0] = Rol_Ptr[0]; 
  output_packet[1] = Rol_Ptr[1]; 
  output_packet[2] = Rol_Ptr[2]; 
  output_packet[3] = Rol_Ptr[3]; 
  output_packet[4] = Pit_Ptr[0];
  output_packet[5] = Pit_Ptr[1];
  output_packet[6] = Pit_Ptr[2];
  output_packet[7] = Pit_Ptr[3];
  output_packet[8] = Yaw_Ptr[0];
  output_packet[9] = Yaw_Ptr[1];
  output_packet[10] = Yaw_Ptr[2];
  output_packet[11] = Yaw_Ptr[3];
  
  Wire.write(output_packet, PACKET_SIZE);
}






