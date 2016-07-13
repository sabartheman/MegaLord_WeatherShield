/*
Purpose: Created for use with the Ublox GPS 
units to configure them into high altitude mode 
utilizing UBX protocol and parse the resulting GPS data.

Author: Dylan Trafford	Creation Date: 07/08/2015
Credit to Scott Miller and Adafruit Industries for code resources

Note: This library is meant to emulate the functionality
of the Adafruit GPS library and uses code from:
	"Written by Limor Fried/Ladyada for Adafruit Industries"
as to replace the GlobalTop GPS with a Ublox GPS module.


Tested with UBLOX PAM7Q-0-000 module using a 4050 buffer for level shifting the UART lines
*/

#ifdef __AVR__
  // Only include software serial on AVR platforms (i.e. not on Due).
  #include <SoftwareSerial.h>
#endif
#include <MSGC_GPS.h>

// how long are max NMEA lines to parse?
#define MAXLINELENGTH 120

boolean Ublox_GPS::parse(char *nmea) {
  // do checksum check

  // first look if we even have one
  if (nmea[strlen(nmea)-4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea)-3]) * 16;
    sum += parseHex(nmea[strlen(nmea)-2]);
    
    // check checksum 
    for (uint8_t i=1; i < (strlen(nmea)-4); i++) {
      sum ^= nmea[i];
    }
    if (sum != 0) {
      // bad checksum :(
      //return false;
    }
  }
  int32_t degree;
  long minutes;
  char degreebuff[10];
  // look for a few common sentences
  if (strstr(nmea, "$GPGGA")) {
    // found GGA
    char *p = nmea;
    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
      latitudeDegrees += int(latitude/100);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else return false;
    }
    
    // parse out longitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
      longitudeDegrees += int(longitude/100);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else return false;
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      fixquality = atoi(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      satellites = atoi(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      HDOP = atof(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      altitude = atof(p);
    }
    
    p = strchr(p, ',')+1;
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      geoidheight = atof(p);
    }
    return true;
  }
  if (strstr(nmea, "$GPRMC")) {
   // found RMC
    char *p = nmea;

    // get time
    p = strchr(p, ',')+1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    p = strchr(p, ',')+1;
    // Serial.println(p);
    if (p[0] == 'A') 
      fix = true;
    else if (p[0] == 'V')
      fix = false;
    else
      return false;

    // parse out latitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      long degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      long minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
      latitudeDegrees += int(latitude/100);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else return false;
    }
    
    // parse out longitude
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
      longitudeDegrees += int(longitude/100);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else return false;
    }
    // speed
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      speed = atof(p);
    }
    
    // angle
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      angle = atof(p);
    }
    
    p = strchr(p, ',')+1;
    if (',' != *p)
    {
      uint32_t fulldate = atof(p);
      day = fulldate / 10000;
      month = (fulldate % 10000) / 100;
      year = (fulldate % 100);
    }
    // we dont parse the remaining, yet!
    return true;
  }

  return false;
}

char Ublox_GPS::read(void) {
  char c = 0;

#ifdef __AVR__
  if(gpsSwSerial) {
    if(!gpsSwSerial->available()) return c;
    c = gpsSwSerial->read();
  } else 
#endif
  {
    if(!gpsHwSerial->available()) return c;
    c = gpsHwSerial->read();
  }

  //Serial.print(c);

//  if (c == '$') {         //please don't eat the dollar sign - rdl 9/15/14
//    currentline[lineidx] = 0;
//    lineidx = 0;
//  }
  if (c == '\n') {
    currentline[lineidx] = 0;

    if (currentline == line1) {
      currentline = line2;
      lastline = line1;
    } else {
      currentline = line1;
      lastline = line2;
    }

    //Serial.println("----");
    //Serial.println((char *)lastline);
    //Serial.println("----");
    lineidx = 0;
    recvdflag = true;
  }

  currentline[lineidx++] = c;
  if (lineidx >= MAXLINELENGTH)
    lineidx = MAXLINELENGTH-1;

  return c;
}
#ifdef __AVR__
// Constructor when using SoftwareSerial or NewSoftSerial
#if ARDUINO >= 100
Ublox_GPS::Ublox_GPS(SoftwareSerial *ser)
#else
Ublox_GPS::Ublox_GPS(NewSoftSerial *ser) 
#endif
{
  common_init();     // Set everything to common state, then...
  gpsSwSerial = ser; // ...override gpsSwSerial with value passed.
}
#endif

// Constructor when using HardwareSerial
Ublox_GPS::Ublox_GPS(HardwareSerial *ser) {
  common_init();  // Set everything to common state, then...
  gpsHwSerial = ser; // ...override gpsHwSerial with value passed.
}

// Initialization code used by all constructor types
void Ublox_GPS::common_init(void) {
#ifdef __AVR__
  gpsSwSerial = NULL; // Set both to NULL, then override correct
#endif
  gpsHwSerial = NULL; // port pointer in corresponding constructor
  recvdflag   = false;
  lineidx     = 0;
  currentline = line1;
  lastline    = line2;
  boolean gpsStatus[] = {false, false, false, false, false, false, false};
  hour = minute = seconds = year = month = day =
    fixquality = satellites = 0; // uint8_t
  lat = lon = mag = 0; // char
  fix = false; // boolean
  milliseconds = 0; // uint16_t
  latitude = longitude = geoidheight = altitude =
    speed = angle = magvariation = HDOP = 0.0; // float
}


boolean Ublox_GPS::newNMEAreceived(void) {
  return recvdflag;
}

char *Ublox_GPS::lastNMEA(void) {
  recvdflag = false;
  return (char *)lastline;
}

uint8_t Ublox_GPS::parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

boolean Ublox_GPS::waitForSentence(const char *wait4me, uint8_t max) {
  char str[20];

  uint8_t i=0;
  while (i < max) {
    if (newNMEAreceived()) { 
      char *nmea = lastNMEA();
      strncpy(str, nmea, 20);
      str[19] = 0;
      i++;

      if (strstr(str, wait4me))
  return true;
    }
  }

  return false;
}

void Ublox_GPS::begin(uint16_t baud)
{
#ifdef __AVR__
  if(gpsSwSerial) 
    gpsSwSerial->begin(baud);
  else 
    gpsHwSerial->begin(baud);
#endif
	
  delay(10);
}

void Ublox_GPS::config_begin(uint16_t baud, byte *settingsArray )
{
  if(gpsSwSerial) 
    gpsSwSerial->begin(baud);
  else 
    gpsHwSerial->begin(baud);

  // Settings Array Breakdown - Default to byte settingsArray[] = {0x07, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // settingsArray[0] = NavMode
  // settingsArray[1] and settingsArray[2] = Data Rate (Update Frequency)
  // settingsArray[3] and settingsArray[4] and settingsArray[5] = Baud Rate (Communication)
  // settingsArray[6] = GLL messages (0x00 is disabled)
  // settingsArray[7] = GSA messages (0x00 is disabled)
  // settingsArray[8] = GSV messages (0x00 is disabled)
  // settingsArray[9] = RMC messages (0x00 is disabled)
  // settingsArray[10] = VTG messages (0x00 is disabled)
  //byte settingsArray[] = {0x07, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	configureUblox(settingsArray);
  delay(10);
}


void Ublox_GPS::configureUblox(byte *settingsArrayPointer) {
  long starttime = millis();
  byte gpsSetSuccess = 0;
  Serial.println("Configuring u-Blox GPS initial state...");
  //Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setNav[2], sizeof(setNav) - 4);

  //Generate the configuration string for Data Rate
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
  calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);

  //Generate the configuration string for Baud Rate
  byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);

  byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
  byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
  byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

  delay(2500);

  while(gpsSetSuccess < 3)
  {
    Serial.print("Setting Navigation Mode... ");
    sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
    if (gpsSetSuccess == 5) {
      gpsSetSuccess -= 4;
      setBaud(settingsArrayPointer[4]);
      delay(1500);
      byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
      sendUBX(lowerPortRate, sizeof(lowerPortRate));
      if(gpsSwSerial) 
        gpsSwSerial->begin(9600);
      else 
        gpsHwSerial->begin(9600);
      delay(2000);      
    }
    if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
    if (gpsSetSuccess == 10) {gpsStatus[0] = true; Serial.println("Success");}
  }
  if (gpsSetSuccess == 3) Serial.println("Navigation mode configuration failed.");
  gpsSetSuccess = 0;


  while(gpsSetSuccess < 3) {
    Serial.print("Setting Data Update Rate... ");
    sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function      
    if (gpsSetSuccess == 10) {gpsStatus[1] = true; Serial.println("Success");}
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("Data update mode configuration failed.");
  gpsSetSuccess = 0;


  while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
    Serial.print("Deactivating NMEA GLL Messages ");
    sendUBX(setGLL, sizeof(setGLL));
    gpsSetSuccess += getUBX_ACK(&setGLL[2]);
    if (gpsSetSuccess == 10) {gpsStatus[2] = true; Serial.println("Success");}
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA GLL Message Deactivation Failed!");
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
    Serial.print("Deactivating NMEA GSA Messages ");
    sendUBX(setGSA, sizeof(setGSA));
    gpsSetSuccess += getUBX_ACK(&setGSA[2]);
    if (gpsSetSuccess == 10) {gpsStatus[3] = true; Serial.println("Success");}
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA GSA Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
    Serial.print("Deactivating NMEA GSV Messages ");
    sendUBX(setGSV, sizeof(setGSV));
    gpsSetSuccess += getUBX_ACK(&setGSV[2]);
    if (gpsSetSuccess == 10) {gpsStatus[4] = true; Serial.println("Success");}
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA GSV Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
    Serial.print("Deactivating NMEA RMC Messages ");
    sendUBX(setRMC, sizeof(setRMC));
    gpsSetSuccess += getUBX_ACK(&setRMC[2]);
    if (gpsSetSuccess == 10) {gpsStatus[5] = true; Serial.println("Success");}
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA RMC Message Deactivation Failed!");  
  gpsSetSuccess = 0;

  while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
    Serial.print("Deactivating NMEA VTG Messages ");
    sendUBX(setVTG, sizeof(setVTG));
    gpsSetSuccess += getUBX_ACK(&setVTG[2]);
    if (gpsSetSuccess == 10) {gpsStatus[6] = true; Serial.println("Success");}
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA VTG Message Deactivation Failed!");

  gpsSetSuccess = 0;
  if (settingsArrayPointer[4] != 0x25) {
    Serial.print("Setting Port Baud Rate... ");
    sendUBX(&setPortRate[0], sizeof(setPortRate));
    setBaud(settingsArrayPointer[4]);
    Serial.println("Success!");
    delay(500);
  }
  Serial.print("Total Boot Time: ");
  Serial.println((millis()-starttime)/1000);
}

boolean Ublox_GPS::navStatus(void)
{
  return gpsStatus[0];
}

void Ublox_GPS::calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}

void Ublox_GPS::sendUBX(byte *UBXmsg, byte msgLength) {
	for(int i = 0; i < msgLength; i++) {
			if(gpsSwSerial){
				gpsSwSerial->write(UBXmsg[i]);
				gpsSwSerial->flush();}
			else{    
				gpsHwSerial->write(UBXmsg[i]);
				gpsHwSerial->flush();}
  }
		if(gpsSwSerial){ 
			gpsSwSerial->println();
			gpsSwSerial->flush();}
		else{    
			gpsHwSerial->println();
			gpsHwSerial->flush();}
	
			
}


byte Ublox_GPS::getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  boolean headerReceived = false;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
		#ifdef __AVR__
			if(gpsSwSerial) 
				if (gpsSwSerial->available()) {
				  incoming_char = gpsSwSerial->read();
				  if (incoming_char == ackPacket[i]) {
					i++;
				  }
				  else if (i > 2) {
					ackPacket[i] = incoming_char;
					i++;
				  }
				}
				if (i > 9) break;
				if ((millis() - ackWait) > 1500) { //Original Timeout value 1500
				  return 5;
				}
				if (i == 4 && ackPacket[3] == 0x00) {
				  return 1;
				}
			else
				if (gpsHwSerial->available()) {
				  incoming_char = gpsHwSerial->read();
				  if (incoming_char == ackPacket[i]) {
					i++;
				  }
				  else if (i > 2) {
					ackPacket[i] = incoming_char;
					i++;
				  }
				}
				if (i > 9) break;
				if ((millis() - ackWait) > 1500) { //Original Timeout value 1500
          Serial.println("ACK Timeout");
				  return 5;
				}
				if (i == 4 && ackPacket[3] == 0x00) {
				  Serial.println("NAK Received");
          return 1;
				}
			#endif
		
  }

  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;
  }
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    return 10;
        }
  else {
    delay(1000);
    return 1;
  }
}


void Ublox_GPS::setBaud(byte baudSetting) {
	#ifdef __AVR__
		if(gpsSwSerial){
			if (baudSetting == 0x12) gpsSwSerial->begin(4800);
			if (baudSetting == 0x4B) gpsSwSerial->begin(19200);
			if (baudSetting == 0x96) gpsSwSerial->begin(38400);
			if (baudSetting == 0xE1) gpsSwSerial->begin(57600);
			if (baudSetting == 0xC2) gpsSwSerial->begin(115200);
			if (baudSetting == 0x84) gpsSwSerial->begin(230400);
		}
		else
			if (baudSetting == 0x12) gpsHwSerial->begin(4800);
			if (baudSetting == 0x4B) gpsHwSerial->begin(19200);
			if (baudSetting == 0x96) gpsHwSerial->begin(38400);
			if (baudSetting == 0xE1) gpsHwSerial->begin(57600);
			if (baudSetting == 0xC2) gpsHwSerial->begin(115200);
			if (baudSetting == 0x84) gpsHwSerial->begin(230400);
  #endif
}

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