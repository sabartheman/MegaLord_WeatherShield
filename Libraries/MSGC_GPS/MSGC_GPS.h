/*

file : Ublox_GPS.h

Purpose: Created for use with the Ublox GPS 
units to configure them into high altitude mode 
utilizing UBX protocol and parse the resulting GPS data.

Author: Dylan Trafford for Montana Space Grant Consortium's BOREALIS Program
Creation Date: 07/08/2015 Edit Date: 7/10/2015
Credit to Scott Miller and Adafruit Industries for code resources

Note: This library is meant to emulate the functionality
of the Adafruit GPS library and uses code from:
	"Written by Limor Fried/Ladyada for Adafruit Industries"
as to replace the GlobalTop GPS with a Ublox GPS module.

Tested with UBLOX PAM7Q-0-000 module using a 4050 buffer for level shifting the UART lines
*/
#ifndef MSGC_GPS_H
#define MSGC_GPS_H
#define MAXLINELENGTH 120
#ifdef __AVR__
  #if ARDUINO >= 100
    #include <SoftwareSerial.h>
  #else
    #include <NewSoftSerial.h>
  #endif
#endif

#if ARDUINO >= 100
 #include "Arduino.h"
#if defined (__AVR__) && !defined(__AVR_ATmega32U4__)
 #include "SoftwareSerial.h"
#endif
#else
 #include "WProgram.h"
 #include "NewSoftSerial.h"
#endif



class Ublox_GPS {
	public:
		void begin(uint16_t baud);
		void config_begin(uint16_t baud, byte*settingsArray);
		boolean newNMEAreceived(void);
		char *lastNMEA(void);
		boolean parse(char *nmea);
		char read(void);
		boolean navStatus(void);

//(no point in reinventing the wheel)		
#ifdef __AVR__
  #if ARDUINO >= 100 
    Ublox_GPS(SoftwareSerial *ser); // Constructor when using SoftwareSerial
  #else
    Ublox_GPS(NewSoftSerial  *ser); // Constructor when using NewSoftSerial
  #endif
#endif
	Ublox_GPS(HardwareSerial *ser); // Constructor when using HardwareSerial
	  
		uint8_t hour, minute, seconds, year, month, day;
		uint16_t milliseconds;
		// Floating point latitude and longitude value in degrees.
		float latitude, longitude;
		// Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
		// and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
		//   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
		int32_t latitude_fixed, longitude_fixed;
		float latitudeDegrees, longitudeDegrees;
		float geoidheight, altitude;
		float speed, angle, magvariation, HDOP;
		char lat, lon, mag;
		boolean fix;
		uint8_t fixquality, satellites;
	  	
		#ifdef __AVR__
			#if ARDUINO >= 100
				SoftwareSerial *gpsSwSerial;
			#else
				NewSoftSerial  *gpsSwSerial;
			#endif
		#endif
		HardwareSerial *gpsHwSerial;
	private:
		void common_init(void);
		uint8_t parseHex(char c);
		boolean waitForSentence(const char *wait4me, uint8_t max);
		void setBaud(byte baudSetting);
		byte getUBX_ACK(byte *msgID);
		void sendUBX(byte *UBXmsg, byte msgLength); 
		void calcChecksum(byte *checksumPayload, byte payloadSize);
		void configureUblox(byte *settingsArrayPointer) ;

		// we double buffer: read one line in and leave one for the main program
		char line1[MAXLINELENGTH];
		char line2[MAXLINELENGTH];
		// our index into filling the current line
		uint8_t lineidx=0;
		// pointers to the double buffers
		char *currentline;
		char *lastline;
		boolean recvdflag;
		boolean gpsStatus[7];
		

	};
	#endif

// Adafruit Licensing for their portion of the code (Must be attached to distributions)
// This code is not for the Ultimate Shield or the intended chipset stated below
	/***********************************
This is the Adafruit GPS library - the ultimate GPS library
for the ultimate GPS module!

Tested and works great with the Adafruit Ultimate GPS module
using MTK33x9 chipset
    ------> http://www.adafruit.com/products/746
Pick one up today at the Adafruit electronics shop 
and help support open source hardware & software! -ada

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution
****************************************/