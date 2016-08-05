//#include <PN532.h>
//#include <PN532_I2C.h>
#include <Wire.h>
#include <I2C_PN532.h>

//#include <I2C.h>

//PN532_I2C pn532_i2c(Wire);
//PN532 nfc = PN532(pn532_i2c);

//#include <MemoryFree.h>
//#include "mfoc.h";

uint8_t buffer[32];
uint8_t pn532_packetbuffer[64];

volatile byte bufEvent[16];
volatile boolean haveData = false;
volatile byte reg_position;
const byte buf_size = sizeof(bufEvent);

#define DEBUG 1

//http://www.nxp.com/documents/user_manual/141520.pdf

#define PN532_I2C_ADDRESS       (0x48 >> 1)

#define PN532_PREAMBLE 0x00
#define PN532_STARTCODE1 0x00
#define PN532_STARTCODE2 0xFF
#define PN532_POSTAMBLE 0x00

#define PN532_HOSTTOPN532 0xD4
#define PN532_PN532TOHOST             (0xD5)

#define PN532_FIRMWAREVERSION 0x02
#define PN532_GETGENERALSTATUS 0x04
#define PN532_READREGISTER 0x06
#define PN532_WRITEREGISTER 0x08
#define PN532_READGPIO 0x0C
#define PN532_WRITEGPIO 0x0E
#define PN532_SETSERIALBAUDRATE 0x10
#define PN532_SETPARAMETERS 0x12
#define PN532_SAMCONFIGURATION  0x14
#define PN532_POWERDOWN 0X16

#define PN532_INLISTPASSIVETARGET 0x4A
#define PN532_INDATAEXCHANGE 0x40
#define PN532_INJUMPFORDEP 0x56
#define PN532_TGINITASTARGET 0x8C
#define PN532_TGGETDATA 0x86
#define PN532_TGSETDATA 0x8E

#define PN532_PACKBUFFSIZ 64

I2C_PN532 pnd(PN532_I2C_ADDRESS);

uint8_t sendAck(uint8_t *raw/*, uint8_t rawlen, uint16_t timeout*/) {

	const uint16_t timeout = 1000;

	byte pn532ack[] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
	byte pn532response_firmwarevers[] = { 0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03 };

//	byte pn532_packetbuffer[PN532_PACKBUFFSIZ];

	uint8_t ackBuf[sizeof(pn532ack)];

	uint8_t abtRxBuf[5];
	size_t len;

	//https://github.com/leg0/libnfc/blob/master/libnfc/drivers/pn532_uart.c
	// I2C START
//	Wire.beginTransmission(PN532_I2C_ADDRESS);
//	I2c.write(PN532_I2C_ADDRESS, pn532ack, sizeof(pn532ack));
	Serial.write(pn532ack, sizeof(pn532ack));

	//Added preamble
	//const uint8_t pn53x_preamble[3] = { PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2 };
	//if (0 != (memcmp(frameBuf, pn53x_preamble, 3))) {
	//log_put(LOG_GROUP, LOG_CATEGORY, NFC_LOG_PRIORITY_ERROR, "%s", "Frame preamble+start code mismatch");
	//error   libnfc.driver.pn532_uart        Frame preamble+start code mismatch
	const uint8_t pn53x_preamble[3] = { 0x00, 0x00, 0xff };
	memcpy(abtRxBuf, pn53x_preamble, 3);

	uint8_t answerSamConfig[16] = { 0 };

	/**
	 0   0x00      PREAMBLE 1 byte
	 1   0x00 0xFF      START CODE 2 bytes (0x00 and 0xFF),
	 3   0x..      LEN 1 byte indicating the number of bytes in the data field (TFI and PD0 to PDn),
	 4   0x..      LCS 1 Packet Length Checksum LCS byte that satisfies the relation:
	 Lower byte of [LEN + LCS] = 0x00,
	 TFI 1 byte frame identifier, the value of this byte depends on the way of the message
	 5   0xD4      - D4h in case of a frame from the host controller to the PN532,
	 5   0xD5      - D5h in case of a frame from the PN532 to the host controller.
	 DATA LEN-1 bytes of Packet Data Information
	 6   0x..      The first byte PD0 is the Command Code,
	 0x00      DCS 1 Data Checksum DCS byte that satisfies the relation:
	 Lower byte of [TFI + PD0 + PD1 + … + PDn + DCS] = 0x00,
	 0x00      POSTAMBLE 1 byte	 .
	 */
	// (PREAMBLE + START1 + START2 + LEN + LCS) + (TFI + DATA[0...n]) + (DCS + POSTAMBLE)
	uint8_t startDataPosition = buffer[3];

	//normal case
	//                               len  Host2Pn532       SamConfig   Mode(Normal) TimeOut Irq
	// 0x00       0x00     0xFF     0x03  0xFD  0xD4       0x14        0x01         0x17    0x00
	//Output
	// 0xD5 0x15
	//get length of package          len   Host2Pn532       InRelease     Target(0x00 = all)
	// (0x00       0x00     0xFF     0x03  0xFD  0xD4       0x52          0x00 0xDA 0x00
	// Output
	// 0xD5 0x53 0x Status :
	//Added Len
	//  Wire.write(length);
	//  Wire.write(~length + 1);                 // checksum of length
	//  Serial.write(length);
	//  Serial.write(~length+1);
	abtRxBuf[3] = raw[3];
	if (buffer[6] == 0x02)
		abtRxBuf[3] = 0x04 + 2; //Firmware need 4 bytes

	if (buffer[6] == 0x06)
		abtRxBuf[3] = (buffer[3]-2)/2 + 2; //Read Register

	if (buffer[6] == 0x4A)
		abtRxBuf[3] = 0x0A + 2;

	if (buffer[6] == 0x40)
			abtRxBuf[3] = 14;

	abtRxBuf[4] = 256 - abtRxBuf[3]; //abtRxBuf[4] = raw[4]; // New AutoChecksum

	pnd._WriteSingleBlock(abtRxBuf, 5);
//	Wire.write(abtRxBuf, 5);
	Serial.write(abtRxBuf, 5);

	//Code response +1
	uint8_t answerCommand[2] = { buffer[5] + 1/*PN532_PN532TOHOST*/, buffer[6]
			+ 1 };

	pnd._WriteSingleBlock(answerCommand, 2);
//	Wire.write(answerCommand, 2);
	Serial.write(answerCommand, 2);

	uint8_t btDCS = (256 - buffer[5] - 1/*PN532_PN532TOHOST*/- buffer[6] - 1);
	memset(answerSamConfig, 0, sizeof(answerSamConfig));

	switch (buffer[6]) {

	case 0x14: { //SamConfiguration OK ! Page 89 in3 out 0

		answerSamConfig[0] = buffer[7];
		answerSamConfig[1] = buffer[8];
//		Wire.write(answerSamConfig, 2);
		pnd._WriteSingleBlock(answerSamConfig, 2);

		answerSamConfig[0] = 0x00;
		Serial.write(answerSamConfig, 1);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, 1); //0x16;
		writePostStuff(dataSum);
		//        writeCommand(buffer/*answerSamConfig*/, 3);
		break;
	}

	case 0x16: {        //PowerDown Page 98 in 2 out 1 (status p67)

		answerSamConfig[0] = buffer[7];
		//Wire.write(answerSamConfig, 1);
		pnd._WriteSingleBlock(answerSamConfig, 1);

		answerSamConfig[0] = 0x00; //AnswerStatus
		Serial.write(answerSamConfig, 1);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, 1); //0x14;
		delay(1);
		writePostStuff(dataSum);
		//		  writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x4A: { //InListPassiveTarget Voir Page 115 TODO
		uint8_t passiveTgt[16] = { 0x00 };
		//In the first example, the host controller requires the initialization of
		//one target at 106 kbps type A.
		//		< D4 4A 01 00

		uint8_t send[] = { buffer[7], buffer[8] };

		pnd._WriteSingleBlock(send, 2);
		//		Serial.write(send, 2);
		//		> D5 4B 01 01 04 00 08 04 92 2E 58 32

		// read data packet
		pnd._ReadSingleBlock(passiveTgt, 10);

//		uint8_t passiveTgt[] = {0x01 ,0x01, 0x04, 0x00, 0x08, 0x04, 0x05, 0x67, 0xFB, 0x6A };
		Serial.write(passiveTgt, 10);

		uint8_t dataSum = btDCS - sum_array(passiveTgt, 10);
		writePostStuff(dataSum);
		//		  writeCommand(passiveTgt, sizeof(passiveTgt));
		break;
	}

	case 0x42: { //InCommunicateThru Page 136 in Variable out Variable
		//Real case :
		//InCommunicateThru
		//debug   libnfc.bus.uart_win32   TX: 00 00 ff 04 fc d4 42 e0 50 ba 00

		uint8_t ict[] = { buffer[7], buffer[8] };
//		Wire.write(ict, 2);
		pnd._WriteSingleBlock(ict, 2);
		Serial.write(ict, 2);

		pnd._ReadSingleBlock(answerSamConfig, buffer[3] -2);
		Serial.write(answerSamConfig, buffer[3] -2);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, buffer[3 -2]);
		writePostStuff(dataSum);
		//        writeCommand(buffer, sizeof(firmwareAnswer));
		break;
	}

	case 0x02: { //GetFirmwareVersion Page 73 in 0 out 4 OK
		uint8_t firmwareAnswer[4] = { 0x32, 0x01, 0x06, 0x07 };

		firmwareAnswer[0] = 0x32;
		firmwareAnswer[1] = 0x01;
		firmwareAnswer[2] = 0x06;
		firmwareAnswer[3] = 0x07;

		pnd._WriteSingleBlock(firmwareAnswer, 4);
		Serial.write(firmwareAnswer, 4);

		uint8_t dataSum = btDCS - sum_array(firmwareAnswer, 4); //0xE8;
		writePostStuff(dataSum);
		//        writeCommand(buffer, sizeof(firmwareAnswer));
		break;
	}

	case 0x00: { //Diagnose Page 69 OK

		//const byte_t pncmd_communication_test[]
		//= { 0x00,0x00,0xff,0x09,0xf7,0xd4,0x00,0x00,'l','i','b','n','f','c',0xbe,0x00 };
		//if (buffer[6] == 0x00) {
		//const byte_t attempted_result[]
		//= { 0x00,0x00,0xff,0x00,0xff,0x00,0x00,0x00,0xff,0x09,0xf7,0xD5,0x01,
		//0x00,'l','i','b','n','f','c',0xbc,0x00};		//NumTest
		uint8_t comTest[7] = { 0 };
		if (buffer[7] == 0x00) {  // Communication line Test
			memcpy(comTest, (char*) buffer + 7, 7/*buffer[3] -3*/);
			// OutParam consists of NumTst concatenate with InParam.
			pnd._WriteSingleBlock(comTest, 7/*sizeof(comTest)*/);
			Serial.write(comTest, 7/*sizeof(comTest)*/);
		}
		if (buffer[7] == 0x01) {
			// ROM Test
		}
		if (buffer[7] == 0x02) {
			// RAM Test
			//This test is for checking RAM; 768 bytes of XRAM and 128 bytes of IDATA
		}

		uint8_t dataSum = btDCS - sum_array(comTest, 7); //0xBC;
		writePostStuff(dataSum);
		//       writeCommand(comTest, sizeof(comTest));
		break;
	}

	case 0x32: { //RFConfiguration Page 101
		uint8_t fake1[1] = { 0x00 };
		uint8_t fake2[2] = { 0x00, 0x00 };
		uint8_t fake3[3] = { 0x00, 0x00, 0x00 };
		uint8_t fake4[4] = { 0x00, 0x00, 0x00, 0x00 };

		if (buffer[7] == 0x01) { //CfgItem RF field          1 byte
			uint8_t RFfield[] = { buffer[8] };
			pnd._WriteSingleBlock(RFfield, 1);
			Serial.write(fake2, sizeof(fake2));
		}
		if (buffer[7] == 0x02) { //CfgItem Various timings   3 bytes
			uint8_t variousTiming[] = { buffer[8], buffer[9], buffer[10] };
			pnd._WriteSingleBlock(variousTiming, 3);
			Serial.write(fake2, sizeof(fake2));
		}
		if (buffer[7] == 0x04) { //CfgItem MaxRtyCOM         1 byte
			uint8_t maxRtyCom[] = { buffer[8] };
			pnd._WriteSingleBlock(maxRtyCom, 1);
			Serial.write(fake2, sizeof(fake2));
		}
		if (buffer[7] == 0x05) { //CfgItem MaxRetries        3 bytes
			uint8_t maxRetries[] = { buffer[8], buffer[9], buffer[10] };
			pnd._WriteSingleBlock(maxRetries, 3);
			Serial.write(fake4, sizeof(fake4));
		}
		if (buffer[7] == 0x0A) { //CfgItem Analog settings for the baudrate 106 kbps type A (11 bytes)
			uint8_t configData106[] = { 0x59, 0xF4, 0x3F, 0x11, 0x4D, 0x85, 0x61, 0x6F, 0x26, 0x62, 0x87};
			pnd._WriteSingleBlock(configData106, sizeof(configData106));
			Serial.write(fake2, sizeof(fake2));
		}
		if (buffer[7] == 0x0B) { //CfgItem Analog settings for the baudrate 212/424 kbps (8 bytes)
			uint8_t configData212[] = { 0x69, 0xFF, 0x3F, 0x11, 0x41, 0x85, 0x61, 0x6F};
			pnd._WriteSingleBlock(configData212, sizeof(configData212));
			Serial.write(fake2, sizeof(fake2));
		}
		if (buffer[7] == 0x0C) { //CfgItem Analog settings for the type B (3 bytes)
			uint8_t analogB[] = { buffer[8], buffer[9], buffer[10] };
			pnd._WriteSingleBlock(analogB, 3);
			Serial.write(fake2, sizeof(fake2));
		}
		if (buffer[7] == 0x0D) { //CfgItem Analog settings for baudrates 212/424 and 848 kbps with ISO/IEC14443-4 protocol (9 bytes)
			uint8_t analog212[] = { 0x85, 0x15, 0x8A, 0x85, 0x08, 0xB2, 0x85, 0x01, 0xDA};
			pnd._WriteSingleBlock(analog212, sizeof(analog212));
			Serial.write(fake2, sizeof(fake2));
		}

		uint8_t dataSum = btDCS - sum_array(fake4, 4); //0xF8;
		writePostStuff(dataSum);
		//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x52: { //InRelease OK !
		answerSamConfig[0] = 0x00;
		pnd._WriteSingleBlock(answerSamConfig, 1);
		Serial.write(answerSamConfig, 1);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, 4); //0xD8;
		writePostStuff(dataSum);
		//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x60: { //InAutoPoll Page 144 in out TODO
		//InAutoPoll
		//		debug   libnfc.chip.pn53x       No timeout
		//		debug   libnfc.bus.uart_win32   TX: 00 00 ff 0a f6 d4 60 14 02 20 10 03 11 12 04 5c 00

		uint8_t PollNr[] = { buffer[7] }; //14
		pnd._WriteSingleBlock(PollNr, 1);

		//specifies the number of polling (one polling is a polling for each Type j)
		//		0x01 – 0xFE : 1 up to 254 polling
		//		0xFF : Endless polling.
		uint8_t Period[] = { buffer[8] }; //02
		pnd._WriteSingleBlock(Period, 1);

		//(0x01 – 0x0F) indicates the polling period in units of 150 ms
		uint8_t Type_1[] = { buffer[9] }; //20
		pnd._WriteSingleBlock(Type_1, 1);

		//indicates the mandatory target type to be polled at the 1st time
		// 10 03 11 12 04
//		byte type = 0;
//		for (int i = 0; i < 5 ; i++) {
//			type = buffer[10 + i];
//			Wire.write(type);
//			Serial.write(type);
//		}

		//http://www.gammon.com.au/forum/?id=10896&reply=10#reply10
		//Master: request information from slave
//		int j= 0;
//		if (Wire.requestFrom(PN532_I2C_ADDRESS, 0x05))  // if request succeeded
//			{
//				for (byte i = 0; i < 0x05; i++)
//				answerSamConfig[j] = Wire.read();
//			j++;
//		}
		if (Wire.requestFrom(PN532_I2C_ADDRESS, 0x0F))
		{
			for (int i = 0; i < len; i++)
			{
				answerSamConfig[i] = Wire.read(); // Read the 10 Bytes from Slave
			}
		}
//		delay(10);
//		if (haveData)
//		{
//			memcpy(answerSamConfig, (byte*)bufEvent, sizeof(answerSamConfig));
//			Serial.write(answerSamConfig, sizeof(answerSamConfig));
//			haveData = false; //TODO
//		}

//		answerSamConfig[0] = 0x01;
//		answerSamConfig[1] = 0x20;
//		answerSamConfig[2] = 0x00;
//		Wire.write(answerSamConfig, 5);
//		Serial.write(answerSamConfig, 5);

//In this second example, the host controller polls for 106 kbps targets.
//One Mifare card and a DEP compliant card are present.
		byte autoPollAnswer[] = { 0x01, 0x10, 0x09, 0x01, 0x04, 0x00, 0x08, 0x04, 0x92, 0x2E,
				0x58, 0x32,	0x40, 0x18, 0x02, 0x00, 0x08, 0x40, 0x04, 0x08, 0x12,
				0x34, 0x56,	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
				0x99, 0x00, 0x00, 0x00, 0x09, 0x01 };
		pnd._WriteSingleBlock(autoPollAnswer, sizeof(autoPollAnswer));
		Serial.write(autoPollAnswer, 8);

		uint8_t dataSum = btDCS - sum_array(autoPollAnswer, 8);
		writePostStuff(dataSum);
		//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x12: { //SetParameters Page 85 in 1 out 0 OK
		answerSamConfig[0] = buffer[7];
		pnd._WriteSingleBlock(answerSamConfig, 1); //Testing Flags is a bit-field byte which individual definition
		Serial.write(answerSamConfig, 1);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, 1); //0x04;
		writePostStuff(dataSum);
		//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x06: { //ReadRegister Page 76 OK FAKE
		uint8_t address;
		uint8_t value[16] = { 0 };
		int j = 0;

		memset(answerSamConfig, 0, sizeof(answerSamConfig));
		for (int i = 0; i <= buffer[3] - 2; i += 2) {
			memcpy(&address, (int*) abtRxBuf + 7 + i, 2);

			answerSamConfig[j] = pnd.readAddressByte(address);

			j++;
		}

		Serial.write(answerSamConfig, abtRxBuf[3] -2);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, abtRxBuf[3] -2); //0x24;
		writePostStuff(dataSum);
		//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x08: { //WriteRegister Page 78 OK
		uint16_t address;
		int value;

		uint8_t dataSum = btDCS;

		for (int i = 0; i < abtRxBuf[3] - 2; i += 3) {
			memcpy(&address, (int*) abtRxBuf + 7 + i, 2);
			memcpy(&value, (int*) abtRxBuf + 9 + i, 1);

			pn532_packetbuffer[0/*1*/] = buffer/*abtRxBuf*/[ 7 + i ];
			pn532_packetbuffer[1/*1*/] = buffer/*abtRxBuf*/[ 8 + i ];
			pn532_packetbuffer[2/*1*/] = buffer/*abtRxBuf*/[ 9 + i ];

			pnd.writeAddressByte(address, value);

			Serial.write(pn532_packetbuffer, 3); // ?

			dataSum -= sum_array(pn532_packetbuffer, 3);
		}

		writePostStuff(dataSum);
		//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}

	case 0x44: {        //InDeselect
			answerSamConfig[0] = buffer[7];
			pnd._WriteSingleBlock(answerSamConfig, 1);

			answerSamConfig[0] = 0x00; //AnswerStatus
			Serial.write(answerSamConfig, 1);

			uint8_t dataSum = btDCS - sum_array(answerSamConfig, 1); //0x14;
			writePostStuff(dataSum);
			//		  writeCommand(answerSamConfig, sizeof(answerSamConfig));
			break;
		}
	case 0x40: {        //InDataExchange page 127 page 130 Mifare TODO
		//debug   libnfc.chip.pn53x       InDataExchange
		//debug   libnfc.bus.uart_win32
		//TX: 00 00 ff 0f f1 d4 40 01 60 3f aa bb cc dd ee ff 05 67 fb 6a 80 00

		uint8_t InDataX[12] = { 0 };
		memcpy(InDataX, (char*) buffer + 8, 12/*buffer[3] -2*/);
		pnd._WriteSingleBlock(InDataX, 12/*sizeof(comTest)*/);

		Serial.write(InDataX, 12/*sizeof(comTest)*/);

		//		answerSamConfig[0] = 0x00; //AnswerStatus
		//		Serial.write(answerSamConfig, 1);

		uint8_t dataSum = btDCS - sum_array(InDataX, 12); //0x14;
		writePostStuff(dataSum);
		//		  writeCommand(answerSamConfig, sizeof(answerSamConfig));
		break;
	}
	default: {
		answerSamConfig[0] = 0x7F;

		pnd._WriteSingleBlock(answerSamConfig, 1);
		Serial.write(answerSamConfig, 1);

		uint8_t dataSum = btDCS - sum_array(answerSamConfig, 1); //0xD8;
		writePostStuff(dataSum);
//        writeCommand(answerSamConfig, sizeof(answerSamConfig));
	}
	}

	return 0; // ack'd command
}

// http://forum.arduino.cc/index.php?topic=329383.0
// WRITE COMMAND
int8_t writeCommand(uint8_t* data, uint8_t dataLen) {
//    D("    PN532 writeCommand()");

	writePreStuff(dataLen);

	uint8_t dataSum = 0;
	int i = 0;
	for (i = 0; i < dataLen; i++) {
		dataSum += data[i];
		//Wire.write(data[i]);
		Serial.write(data[i]);
	}

	writePostStuff(dataSum);

	return 1;
}

// WRITE PREAMBLE AND CHEKS
int8_t writePreStuff(uint8_t len) {
	//  D("    PN532 writePreStuff()");

//  Wire.prepareWrite(PN532_I2C_ADDRESS);
//	Wire.write(PN532_PREAMBLE);
	Serial.write(PN532_PREAMBLE);
//	Wire.write(PN532_STARTCODE1);
	Serial.write(PN532_STARTCODE1);
//	Wire.write(PN532_STARTCODE2);
	Serial.write(PN532_STARTCODE2);
//	Wire.write(len + 1);
	Serial.write(len + 1);
//	Wire.write(~(len));
	Serial.write(~(len));
}

// WRITE POSTAMBLE STUFF AND CHECK
int8_t writePostStuff(uint8_t dataSum) {

	byte Sum[] = { dataSum };
	Wire.write(dataSum);
//	I2c.write(PN532_I2C_ADDRESS, Sum, 1);
	Serial.write(dataSum);

	Wire.write(PN532_POSTAMBLE);
//	I2c.write(PN532_I2C_ADDRESS, PN532_POSTAMBLE, sizeof(PN532_POSTAMBLE));
	Serial.write(PN532_POSTAMBLE);

//	Wire.endTransmission();
}


int sum_array(uint8_t a[], int num_elements) {
	int i, sum = 0;
	for (i = 0; i < num_elements; i++) {
		sum = sum + a[i];
	}
	return (sum);
}

// called by interrupt service routine when incoming data arrives
void receiveEvent(int howMany)
{
	int receiveByte = 0; // set index to 0
		while(Wire.available()) // loop through all incoming bytes
		{
			bufEvent[receiveByte++] = Wire.read(); // receive byte as a character
		}
		haveData = true;
}  // end of receiveEvent

void setup() {
	Serial.begin(115200); //115200, 230400, 460800
	Serial.flush();

//	nfc.begin();

	Wire.begin(PN532_I2C_ADDRESS);
// set up receive handler  (in setup)
//	Wire.onReceive (receiveEvent);  // interrupt handler for incoming messages

    pnd.begin();

}

void loop() {
	/**
	 Description
	 Get the number of bytes (characters) available for reading from the serial port.
	 This is data that's already arrived and stored in the serial receive buffer (which holds 64 bytes).
	 Returns
	 the number of bytes available to read
	 */
	int b = Serial.available();
	if (b >= 5) {
		Serial.readBytes((char*) buffer, 5);
		if (buffer[0] == 0x55 && buffer[1] == 0x55) {
			//  #if DEBUG
			//              Serial.println("Wakeup case");
			//  #endif
			//ignore wakeup from libnfc
			Serial.readBytes((char*) buffer + 5, 11);
			//handle wake up case
			//send raw command to pn532
			//get length of package :
			// (PREAMBLE + START + LEN + LCS) + (TFI + DATA[0...n]) + (DCS + POSTAMBLE)
			// uint8_t l = buffer[8] + 2;
			// while (Serial.available() < l); //wait the command
		} else {
			//#if DEBUG
			//              Serial.println("Normal case");
			//#endif

			//read command from uart
			uint8_t len = buffer[3] + 2;

			uint8_t br = len;
			uint8_t* bufptr = buffer + 5;
			while (br) {
				if (Serial.available()) {
					*bufptr++ = Serial.read();
					--br;
				}
			}

			//send raw command to pn532
			sendAck(buffer/*, startPosition + 5*/);
		}
	}
}
