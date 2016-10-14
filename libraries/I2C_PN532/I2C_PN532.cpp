//
//    FILE: I2C_PN532.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 1.2.03
// PURPOSE: I2C_eeprom library for Arduino with EEPROM 24LC256 et al.
//
// HISTORY:
// 0.1.00 - 2011-01-21 initial version
// 0.1.01 - 2011-02-07 added setBlock function
// 0.2.00 - 2011-02-11 fixed 64 bit boundary bug
// 0.2.01 - 2011-08-13 _readBlock made more robust + return value
// 1.0.00 - 2013-06-09 support for Arduino 1.0.x
// 1.0.01 - 2013-11-01 fixed writeBlock bug, refactor
// 1.0.02 - 2013-11-03 optimize internal buffers, refactor
// 1.0.03 - 2013-11-03 refactor 5 millis() write-latency
// 1.0.04 - 2013-11-03 fix bug in readBlock, moved waitEEReady()
//                     -> more efficient.
// 1.0.05 - 2013-11-06 improved waitEEReady(),
//                     added determineSize()
// 1.1.00 - 2013-11-13 added begin() function (Note breaking interface)
//                     use faster block Wire.write()
//                     int casting removed
// 1.2.00 - 2014-05-21 Added support for Arduino DUE ( thanks to Tyler F.)
// 1.2.01 - 2014-05-21 Refactoring
// 1.2.02 - 2015-03-06 stricter interface
// 1.2.03 - 2015-05-15 bugfix in _pageBlock & example (thanks ifreislich )
//
// Released to the public domain
//

#include <I2C_PN532.h>

#if defined(ARDUINO) && ARDUINO >= 100
    #define WIRE_WRITE Wire.write
    #define WIRE_READ  Wire.read
#else
    #define WIRE_WRITE Wire.send
    #define WIRE_READ  Wire.receive
#endif


I2C_PN532::I2C_PN532(const uint8_t deviceAddress)
{
	I2C_PN532(deviceAddress, I2C_PN532_PAGESIZE);
}

I2C_PN532::I2C_PN532(const uint8_t deviceAddress, const unsigned int deviceSize)
{
    _deviceAddress = deviceAddress;

   // PN532 RAM -> 768 bytes of XRAM
    if (deviceSize <= 256)
    {
        this->_isAddressSizeTwoWords = false;
//        this->_pageSize = 8;
    }
//    else if (deviceSize <= 256 * 8)
//    {
//        this->_isAddressSizeTwoWords = false;
//        this->_pageSize = 16;
//    }
    else
    {
        this->_isAddressSizeTwoWords = true;
//        this->_pageSize = 32;
    }
}

void I2C_PN532::begin()
{
    Wire.begin();
    _lastWrite = 0;

// TWBR is not available on Arduino Due
//    #if ARDUINO >= 158
//      Wire.setClock(400 * 1000);
//	#else
//      TWBR = (F_CPU / (400 * 1000) - 16) / 2;
// 0=1000 1=888 2=800 8=500
// 12=400KHz  24=250 32=200  72=100  152=50
// F_CPU/16+(2*TWBR) // TWBR is a uint8_t
//	#endif
}

int I2C_PN532::writeAddressByte(const uint16_t memoryAddress, const uint8_t data)
{
    int rv = _WriteAddressBlock(memoryAddress, &data, 1);
    return rv;
}

uint8_t I2C_PN532::readAddressByte(const uint16_t memoryAddress)
{
    uint8_t rdata;
    _ReadAddressBlock(memoryAddress, &rdata, 1);
    return rdata;
}

////////////////////////////////////////////////////////////////////
//
// PRIVATE
//


// supports one and 2 bytes addresses
void I2C_PN532::_beginTransmission(const uint16_t memoryAddress)
{
  Wire.beginTransmission(_deviceAddress);

//  if (this->_isAddressSizeTwoWords)
//  {
  WIRE_WRITE((memoryAddress >> 8) & 0xFF);    // Address High Byte
//  }

  WIRE_WRITE((memoryAddress >> 0 ) & 0xFF);  // Address Low Byte (or only byte for chips 16K or smaller that only have one-word addresses)
}


void I2C_PN532::_beginSingleTransmission()
{
  Wire.beginTransmission(_deviceAddress);
}

// pre: length <= this->_pageSize  && length <= I2C_TWIBUFFERSIZE;
// returns 0 = OK otherwise error
int I2C_PN532::_WriteAddressBlock(const uint16_t memoryAddress, const uint8_t* buffer, const uint8_t length)
{
    waitPN532Ready();

    this->_beginTransmission(memoryAddress);

    WIRE_WRITE(buffer, length);

    int rv = Wire.endTransmission();
    _lastWrite = micros();
    return rv;
}

int I2C_PN532::_WriteSingleBlock(const uint8_t* buffer, const uint8_t length)
{
    waitPN532Ready();

    this->_beginSingleTransmission();

    WIRE_WRITE(buffer, length);

    int rv = Wire.endTransmission();
    _lastWrite = micros();
    return rv;
}

uint8_t I2C_PN532::_ReadSingleBlock(uint8_t* buffer, const uint8_t length)
{
    waitPN532Ready();

    this->_beginSingleTransmission();

    delay(5);

    int rv = Wire.endTransmission();
    if (rv != 0) return 0;  // error

    delay(5);

    Wire.requestFrom(_deviceAddress, length);
    uint8_t cnt = 0;
//    uint32_t before = millis();
    while ((cnt < length) /*&& ((millis() - before) < I2C_PN532_TIMEOUT)*/)
    {
        if (Wire.available()) buffer[cnt++] = WIRE_READ();
    }
    return cnt;
}

// pre: buffer is large enough to hold length bytes
// returns bytes read
uint8_t I2C_PN532::_ReadAddressBlock(const uint16_t memoryAddress, uint8_t* buffer, const uint8_t length)
{
    waitPN532Ready();

    this->_beginTransmission(memoryAddress);

    delay(5);

    int rv = Wire.endTransmission();
    if (rv != 0) return 0;  // error

    delay(5);

    Wire.requestFrom(_deviceAddress, length);
    uint8_t cnt = 0;
//    uint32_t before = millis();
    while ((cnt < length) /*&& ((millis() - before) < I2C_PN532_TIMEOUT)*/)
    {
        if (Wire.available()) buffer[cnt++] = WIRE_READ();
    }
    return cnt;
}

void I2C_PN532::waitPN532Ready()
{
#define I2C_WRITEDELAY  10000

    // Wait until EEPROM gives ACK again.
    // this is a bit faster than the hardcoded 10 milliSeconds
    while ((micros() - _lastWrite) <= I2C_WRITEDELAY)
    {
        Wire.beginTransmission(_deviceAddress);
        int x = Wire.endTransmission();
        if (x == 0) break;
    }
}

// END OF FILE
