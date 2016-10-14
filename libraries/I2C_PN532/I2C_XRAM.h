/*
 * XRAMi2c.h
 *
 *  Created on: 3 août 2016
 *      Author: Pascal
 */

#ifndef LIBRARIES_I2C_PN532_I2C_XRAM_H_
#define LIBRARIES_I2C_PN532_I2C_XRAM_H_

#define DEVICE 0x24 //0x48
//this is the device ID from the datasheet of the PN532

//in the normal write anything the eeaddress is incrimented after the writing
//of each byte. The Wire library does this behind the scenes.

template<class T> int eeWrite(word ee, const T& value) {
	const byte* p = (const byte*) (const void*) &value;
	int i;
	Wire.beginTransmission(DEVICE);
//	Wire.write(PN532_COMMAND_WRITEREGISTER);
	Wire.write((int) highByte(ee)/*(ee >> 8)*/); // MSB
	Wire.write((int) lowByte(ee) /*(ee & 0xFF)*/); // LSB
	for (i = 0; i < sizeof(value); i++)
		Wire.write(*p++);
	Wire.endTransmission();
	return i;
}

template<class T> int eeRead(word ee, T& value) {
	byte* p = (byte*) (void*) &value;
	int i;
	Wire.beginTransmission(DEVICE);
//	Wire.write(PN532_COMMAND_READREGISTER);
	Wire.write((int) highByte(ee)/*(ee >> 8)*/); // MSB
	Wire.write((int) lowByte(ee) /*(ee & 0xFF)*/); // LSB
	Wire.endTransmission();
	Wire.requestFrom(DEVICE, sizeof(value));
	for (i = 0; i < sizeof(value); i++)
		if (Wire.available())
			*p++ = Wire.read();
	return i;
}

#endif /* LIBRARIES_I2C_PN532_I2C_XRAM_H_ */
