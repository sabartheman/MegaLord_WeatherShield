
/*
  Author: Dylan Trafford 7/10/2015 for MSGC BOREALIS PROGRAM at Montana State University
  Purpose: To test the Ublox PAM-7Q GPS on an Arduino Mega 2560
    and verify library functionality for MSGC_GPS. The MSGC library
    was created to allow reconfiguration of the Ublox GPS modules for
    high alititude usage.
    
  Note: We ran into a SRAM issue using Arudino Unos R3 and are running
    this on a Arduino Mega 2560 to work around this.
*/

#include <SoftwareSerial.h>
#include <MSGC_GPS.h>


//SoftwareSerial mySerial(8,7)  Can be used on Uno if wired to match Adafruit Shield
//Mega uses hardware Serial
Ublox_GPS GPS(&Serial1); 
//Ublox_GPS GPS(&mySerial) for Software Serial on Uno 

// Settings Array Breakdown - Default to "byte settingsArray[] = {0x07, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};" which gives you basic information at altitude
// settingsArray[0] = NavMode
// settingsArray[1] and settingsArray[2] = Data Rate (Update Frequency)
// settingsArray[3] and settingsArray[4] and settingsArray[5] = Baud Rate (Communication)
// settingsArray[6] = GLL messages Enable/Disable (0x00 is disabled)
// settingsArray[7] = GSA messages Enable/Disable(0x00 is disabled)
// settingsArray[8] = GSV messages Enable/Disable(0x00 is disabled)
// settingsArray[9] = RMC messages Enable/Disable(0x00 is disabled)
// settingsArray[10] = VTG messages Enable/Disable(0x00 is disabled) 
byte settingsArray[] = {0x07, 0xE8, 0x03, 0x80, 0x25, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01};
volatile long timecheck = 0;

void setup() {
  Serial.begin(115200);
  GPS.config_begin(9600,settingsArray);
}

void loop() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (millis() > timecheck){ 
    Serial.println("----------------------------");
    Serial.print("UTC: "); Serial.print(GPS.hour); Serial.print(":"); Serial.print(GPS.minute); Serial.print(":"); Serial.println(GPS.seconds);
    Serial.print("Lat: "); Serial.println(GPS.latitudeDegrees,7);
    Serial.print("Lon: "); Serial.println(GPS.longitudeDegrees,7);
    Serial.print("Alt: "); Serial.println(GPS.altitude,0);
    timecheck = millis() + 2000;
  }
}
