// ############################################################################################## //
// ################################ SPI read/write functions #################################### //
// ############################################################################################## //

// --- Function for SPI reading one byte from sensor
// reg        : MPU-6000 register number to read from
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > register contents
byte SPIread(byte reg, int ChipSelPin)
{
  digitalWrite(ChipSelPin, LOW);     // select MPU-6000 for SPI transfer (low active)
  SPI.transfer(reg | 0x80);          // reg | 0x80 causes a "1" added as MSB to reg to denote reading from reg i.s.o. writing to it
  byte read_value = SPI.transfer(0x00); // write 8-bits zero to MPU-6000, read the 8-bits coming back from reg at the same time
  digitalWrite(ChipSelPin, HIGH);    // deselect MPU-6000 for SPI transfer
  return read_value;
}

// --- Function for SPI writing one byte to sensor
// reg        : MPU-6000 register number to write to
// data       : data to be written into reg
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > nothing
void SPIwrite(byte reg, byte data, int ChipSelPin)
{
  digitalWrite(ChipSelPin, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(ChipSelPin, HIGH);
}

// --- Function for SPI reading one bit from sensor
// reg        : MPU-6000 register number to read from
// bitNum     : bit number in the register to read - 7 (MSB) to 0 (LSB)
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > byte 0x00 if bit is 0, otherwise byte with a 1 at bitNum (rest 0's)
byte SPIreadBit(byte reg, byte bitNum, int ChipSelPin)
{
  byte byte_value = SPIread(reg, ChipSelPin);
  byte bit_value  = byte_value & (1 << bitNum); // AND result from register byte value and byte with only one "1" at place of bit to return (rest "0"'s)
  return bit_value;
}

//--- Function for SPI writing one bit to sensor
// reg        : MPU-6000 register number to write to
// bitNum     : bit number in the register to write to - 7 (MSB) to 0 (LSB)
// databit    : bit value to be written into reg - false or 0 | true or non-zero (1 will be logical)
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > nothing
//
// first read byte, then insert bit value, then write byte:
// otherwise all other bits will be written 0, this may trigger unexpected behaviour
void SPIwriteBit(byte reg, byte bitNum, byte databit, int ChipSelPin)
{
  byte byte_value = SPIread(reg, ChipSelPin);
  if (databit == 0)
  {
    byte_value = byte_value & ~(1 << bitNum); // AND result from register byte value and byte with only one "0" at place of bit to write (rest "1"'s)
  }
  else // databit is intended to be a "1"
  {
    byte_value = byte_value |  (1 << bitNum); // OR  result from register byte value and byte with only one "1" at place of bit to write (rest "0"'s)
  }
  SPIwrite(reg, byte_value, ChipSelPin);
}

//--- Function for SPI reading multiple bytes to sensor
// read multiple bytes from the same device register, most of the times this
// is the FIFO transfer register (which after each read, is automatically
// loaded with new data for the next read)
// reg        : MPU-6000 register number to write to
// length     : number of bytes to be read
// data       : buffer array (starting with [0]) to store the read data in
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > array of data[0 - length]
void SPIreadBytes(byte reg, unsigned int length, byte *data, int ChipSelPin) 
{
  digitalWrite(ChipSelPin, LOW);
  delay(10); // wait 10 ms for MPU-6000 to react on chipselect (if this is 4 ms or less, SPI.transfer fails)
  SPI.transfer(reg | 0x80); // reg | 0x80 causes a "1" added as MSB to reg to denote reading from reg i.s.o. writing to it

  unsigned int count = 0;
  byte data_bytes_printed = 0;

  for (count = 0; count < length; count ++)
  {
    data[count] = SPI.transfer(0x00);
  }
  digitalWrite(ChipSelPin, HIGH);
}

//--- Function for SPI reading multiple bits from sensor
// reg        : MPU-6000 register number to read from
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
// return     > databits
//
// 01101001 read byte
// 76543210 bit numbers
//    xxx   bitStart = 4, length = 3
//    010   masked
//   -> 010 shifted
byte SPIreadBits(byte reg, byte bitStart, byte length, int ChipSelPin)
{
    byte b = SPIread(reg, ChipSelPin);
    byte mask = ((1 << length) - 1) << (bitStart - length + 1);
    b = b & mask;
    b = b >> (bitStart - length + 1);
    return b;
}

//--- Function for SPI writing multiple bits to sensor
// reg        : MPU-6000 register number to write to
// ChipSelPin : MPU-6000 chip select pin number (in this sketch defined by ChipSelPin1)
//
// bbbbb010 -> data (bits to write - leading 0's)
// 76543210 bit numbers
//    xxx   bitStart = 4, length = 3
// 00011100 mask byte
// 10101111 original reg value (read)
// 10100011 original reg value & ~mask
// 10101011 masked | original reg value
//
// first read byte, then insert bit values, then write byte:
// otherwise all other bits will be written 0, this may trigger unexpected behaviour
void SPIwriteBits(byte reg, byte bitStart, byte length, byte data, int ChipSelPin)
{
  byte byte_value = SPIread(reg, ChipSelPin1);
  byte mask = ((1 << length) - 1) << (bitStart - length + 1); // create mask
  data <<= (bitStart - length + 1); // shift data into correct position
  data &= mask;                     // zero all non-important bits in data (just to make sure)
  byte_value &= ~(mask);            // zero all important bits in existing byte, maintain the rest
  byte_value |= data;               // combine data with existing byte
  SPIwrite(reg, byte_value, ChipSelPin);
}