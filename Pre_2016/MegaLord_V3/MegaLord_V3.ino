#include <MSGC_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdio.h>


Ublox_GPS GPS(&Serial3);
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
boolean usingInterrupt = true;
volatile boolean SDConnect = false;     //boolean to control if SD card writing is available
volatile boolean inertialDetect = false; 
const int chipSelect = 10;              // for microSD card
const int I0 = 29;
const int I1 = 27;
const int I2 = 25;
const int Is0 = 23;  //Cutdown1 Status (HIGH = TRIGGERED, LOW = UnTRIGGERED)
const int Is1 = 24;  //Cutdown2 Status (HIGH = TRIGGERED, LOW = UnTRIGGERED)
const int Is2 = 28;  //Valve Status (HIGH = OPEN, LOW = CLOSED)
const int Is3 = 26;  //System Status (LOW = GOOD, HIGH = ERROR)
boolean cutdown1 = false; //Currently AIRMAIL razor cutdown
boolean cutdown2 = false; //Currently Tow Release cutdown
boolean valve = false; // False = Closed, True = Open
int input = 0; //Iridium Input

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup() {
  //IO Setup
  pinMode(I0, INPUT); //Iridium Output 0 (orange)
  pinMode(I1, INPUT); //Iridium Output 1 (yellow)
  pinMode(I2, INPUT); //Iridium Output 2 (green)
  pinMode(Is3, OUTPUT); //Iridium Input S2 (blue)
  pinMode(Is2, OUTPUT); //Iridium Input S3 (red)
  pinMode(Is1, OUTPUT); //Spare Data Line
  pinMode(Is0, OUTPUT); //Spare Data Line
  digitalWrite(Is3, LOW);
  digitalWrite(Is2, LOW);
  digitalWrite(Is1, LOW);
  digitalWrite(Is0, LOW);
  //Serial Setup
  Serial2.begin(9600); //For Xbee
  Serial2.setTimeout(500); // Timeout for xbee connection
  Serial.begin(115200); //For Serial Monitor
  Serial.println("Booting MEGALORD");
  int xdeg;
  int ydeg;
  int zdeg;
  int orex;
  int orey;
  int orez;
  int Sys;
  int Gyro;
  int Accel;
  int Mag;



  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    inertialDetect = false;
  }
  else {
    inertialDetect = true;
    Serial.println("BNO055 Detected and Started");
    delay(1000);
    bno.setExtCrystalUse(true);
  }
  //SD Setup
  if (!SD.begin(10, 11, 12, 13)) {
    //if(!SD.begin(chipSelect)){
    Serial.println("Micro SD card failure");
    SDConnect = false;
  } else
  {
    Serial.println("Initializing SD Card");
    SDConnect = true;
    const char rebootText[] PROGMEM = "------------------MEGALORD POWERUP------------------\nHour,Min,Sec,Iridium,Lat,Long,Alt,Cut1,Cut2,Valve,LM19(para),LM19(lin),BNO,ExtThermistor,pressure,xaccel,yaccel,zaccel,xdeg,ydeg,zdeg,xore,yore,zore,sys,gyro,accel,mag";
    File logFile = SD.open("datalog1.txt", O_CREAT  | O_WRITE);
    logFile.println(rebootText);
    logFile.close();
    logFile = SD.open("datalog2.txt", O_CREAT  | O_WRITE);
    logFile.println(rebootText);
    logFile.close();
  }
  GPS.config_begin(9600, settingsArray); //For GPS
  useInterrupt(usingInterrupt);
}

SIGNAL(TIMER0_COMPA_vect) { //Already existing 1ms interrupt
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
uint32_t timer2 = millis();


void loop() {
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
  }
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();
  if (timer2 > millis())  timer2 = millis();

  if (millis() - timer2 > 5000) {
    //checkWireless();
    //input = checkIridium();
    timer2 = millis();
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 333) {
    timer = millis(); // reset the timer
    Serial.println("##########################################################################################");
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    Serial.println("------------------------");
    Serial.println("Sensors");
    int pressurecounts = analogRead(A2);
    float outputv_pressure = (5 / 1024.000000) * pressurecounts;
    float pressure = ((outputv_pressure * 15) / 4) - (0.5 * 15 / 4);
    Serial.print("Air Pressure: "); Serial.println(pressure);
    int lm19_counts = analogRead(A3);
    Serial.print(lm19_counts);
    float outputv_lm19 = (5 / 1024.000000) * lm19_counts;
    float internaltemp = -1481.96 + sqrt(2196200 + ((1.8639 - outputv_lm19) / 0.00000388));
    float internaltemp_lin = (outputv_lm19 - (1.8528)) / (-0.01179);
    Serial.print("Internal Temp (LM19lin): "); Serial.println(internaltemp_lin);
    Serial.print("Internal Temp (LM19): "); Serial.println(internaltemp);
    Serial.print("Internal Temp (BNO): "); Serial.println(bno.getTemp());
    float outputv_thermistor0 = analogRead(A0) * (5 / 1024.0000); //Thermistor Voltage
    float outputc_thermistor0 = (5 - outputv_thermistor0) / 100000.0000; //Current
    float outputr_thermistor0 = outputv_thermistor0 / outputc_thermistor0;
    float thermistor0_temp = (1 / 298.15) + (1 / 3950.000) * log(outputr_thermistor0 / 10000.00000); //https://learn.adafruit.com/thermistor/using-a-thermistor
    thermistor0_temp = (1 / thermistor0_temp) - 273;
    Serial.print("Thermistor (A0): "); Serial.println(thermistor0_temp);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> oreo = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    int xdeg = (((gyro.x())*180)/3.14159265359);
    int ydeg = (((gyro.y())*180)/3.14159265359);
    int zdeg = (((gyro.z())*180)/3.14159265359);
    int xore = (oreo.x());
    int yore = (oreo.y());
    int zore = (oreo.z());
    int Sys;
    int Gyro;
    int Accel;
    int Mag;
   
    if (inertialDetect) {
      /* Get a new sensor event */
      sensors_event_t event;
      bno.getEvent(&event);
      /* Display the floating point data 
      Serial.println("Orientation");
      Serial.print("X: ");
      Serial.print(event.orientation.x, 4);
      Serial.print("\tY: ");
      Serial.print(event.orientation.y, 4);
      Serial.print("\tZ: ");
      Serial.print(event.orientation.z, 4);
      Serial.println("");*/
      
      Serial.println("Euler Orientation");
      Serial.print("X: ");
      Serial.print(oreo.x(), 4);
      Serial.print("\tY: ");
      Serial.print(oreo.y(), 4);
      Serial.print("\tZ: ");
      Serial.print(oreo.z(), 4);
      Serial.println("");
      
      Serial.println("Acceleration in m/s/s");
      Serial.print("X: ");
      Serial.print(accel.x(), 2);
      Serial.print(" Y: ");
      Serial.print(accel.y(), 2);
      Serial.print(" Z: ");
      Serial.print(accel.z(), 2);
      Serial.println("");
      Serial.println("Velocity in Degrees/s");
      /*Serial.print("X: ");
      Serial.print(gyro.x(), 2);
      Serial.print(": ");
      Serial.print(gyro.y(), 2);
      Serial.print(": ");
      Serial.print(gyro.z(), 2);
      Serial.println("");
      Serial.println("Degrees");*/
      Serial.print("X: ");
      Serial.print (xdeg);
      Serial.print(" ");
      Serial.print(" Y: ");
      Serial.print (ydeg);
      Serial.print(" ");
      Serial.print(" Z: ");
      Serial.println (zdeg);
      
      uint8_t sys, gyro, accel, mag = 0;
      bno.getCalibration(&sys, &gyro, &accel, &mag);
      Serial.println(F("Calibration "));
      Serial.print("Sys: ");
      Serial.print(sys, DEC);
      Serial.print(F(" "));
      Serial.print("Gyro: ");
      Serial.print(gyro, DEC);
      Serial.print(F(" "));
      Serial.print("Accel: ");
      Serial.print(accel, DEC);
      Serial.print(F(" "));
      Serial.print("Mag: ");
      Serial.println(mag, DEC);
      Serial.println("X = Yaw Y = Pitch Z= Roll");    
      Sys = (sys);
      Gyro = (gyro);
      Accel = (accel);
      Mag = (mag);

        
    
    }
    Serial.println("------------------------");
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location (in degrees): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 4);





      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Iridium Input: "); Serial.println(input);
      if (SDConnect) {
        logData(input, internaltemp, internaltemp_lin, bno.getTemp(), thermistor0_temp, pressure, accel.x(), accel.y(), accel.z(), xdeg, ydeg, zdeg, xore, yore, zore, Sys, Gyro, Accel, Mag);
      }
    }
  }
}


/*int checkIridium(void){
  int input0 = digitalRead(I0);
  int input1 = digitalRead(I1);
  int input2 = digitalRead(I2);
  int combined = (int)(input0*100)+(int)(input1*10) + input2;
  switch(combined){
    case 0:  // 000 - Close valve case
      Serial.println("Requested Valve Closed / System Idle");
      Serial2.write("VCL");
      valve = false;
      break;
    case 1:  // 001 - Fire Poseidon (unused)
      break;
    case 10:  // 010 - Run Cutdown1 Sequence (AIRMAIL)
      Serial.println("Requested Airmail");
      Serial2.write("AIR");
      cutdown1 = true;
      break;
    case 11:  // 011 - Secondary Cutdown
      Serial.println("Requested Secondary Cutdown");
      Serial2.write("CD2");
      break;
    case 100:  // 100 - Open Valve
      Serial.println("Requested Valve Open");
      Serial2.write("VOP");
      valve = true;
      break;
    case 101:  // 101 - Run Cutdown2 Sequence
      Serial.println("Requested Tow Line Release");
      Serial2.write("TOW");
      cutdown2 = true;
      break;
    case 110:  // 110 - Disable Timers (Unused)
      break;
    case 111:  // 111 - Reset Timers (Reworked to a manual switch for reset)
      //Serial.println("Requested Timer Reset");
      //Serial2.write("RES");
      //cutdown1 = false;
      //cutdown2 = false;
      break;
    default:   // Do nothing
      break;
  }
  return combined;
  }


  void checkWireless(void){
  //const int Is0 = 23;  //Cutdown1 Status (HIGH = TRIGGERED, LOW = UnTRIGGERED)
  //const int Is1 = 24;  //Cutdown2 Status (HIGH = TRIGGERED, LOW = UnTRIGGERED)
  //const int Is2 = 28;  //Valve Status (HIGH = OPEN, LOW = CLOSED)
  //const int Is3 = 26;  //System Status (LOW = GOOD, HIGH = ERROR)
   int CHK[10] = {0,0,0,0,0,0,0,0,0,0};
   Serial2.write("CHK");  //Check All for Triggered
   int i = 0;
   while(Serial2.available() > 0){
     CHK[i] = (int)Serial2.read();
     i++;}
   cutdown1 = false;
   digitalWrite(Is0,LOW);
   cutdown2 = false;
   digitalWrite(Is1,LOW);
   valve = false;
   digitalWrite(Is2,LOW);
   for(int z = 0; z < 10; z++){
     if(CHK[z] == '1'){
       cutdown1 = true;
       digitalWrite(Is0,HIGH);}
     if(CHK[z] == '2'){
       cutdown2 = true;
       digitalWrite(Is1,HIGH);}
     if(CHK[z] == '3'){
       valve = true;
       digitalWrite(Is2,HIGH);}
     Serial.print(CHK[z]);
   }
   Serial.println();
   Serial.print("Cutdown 1: "); Serial.print(cutdown1); Serial.print(" Cutdown 2: "); Serial.print(cutdown2); Serial.print(" Valve: "); Serial.println(valve);
   Serial.println("0 = Untriggered/Default State // 1 = Triggered // Open");
   Serial.println();
  }*/
void logData(int input, float internaltemp, float internaltemp_lin, float bnotemp, float thermistor0_temp, float pressure, float xaccel, float yaccel, float zaccel, int xdeg, int ydeg, int zdeg, int xore, int yore, int zore, int Sys, int Gyro, int Accel, int Mag) {
  //File logFile = SD.open("OVRLRD1.txt", FILE_WRITE);
  File logFile = SD.open("datalog1.txt", O_APPEND | O_WRITE);  //Format = Hours,Minutes,Seconds,Input,Latitude,Longitude,Alititude,ValveStatus,PoseidonStatus,CutdownStatus,PressureSensor
  logFile.print(GPS.hour);    // Hours (UTC)
  logFile.print(F(","));
  logFile.print(GPS.minute);  // Minutes (UTC)
  logFile.print(F(","));
  logFile.print(GPS.seconds);  // Seconds (UTC)
  logFile.print(F(","));    
  logFile.print(GPS.milliseconds);  // Milliseconds (UTC)
  logFile.print(F(","));
  logFile.print(input);               // Input (ie command sent via Freeman
  logFile.print(F(","));
  logFile.print(GPS.latitudeDegrees, 7);   // lat
  logFile.print(F(","));
  logFile.print(GPS.longitudeDegrees, 7);  // long
  logFile.print(F(","));
  logFile.print(GPS.altitude);    // alt
  logFile.print(F(","));
  logFile.print(cutdown1);
  logFile.print(F(","));
  logFile.print(cutdown2);
  logFile.print(F(","));
  logFile.print(valve);
  logFile.print(F(","));
  logFile.print(internaltemp);
  logFile.print(F(","));
  logFile.print(internaltemp_lin);
  logFile.print(F(","));
  logFile.print(bnotemp);
  logFile.print(F(","));
  logFile.print(thermistor0_temp);
  logFile.print(F(","));
  logFile.print(pressure);
  logFile.print(F(","));
  logFile.print(xaccel);
  logFile.print(F(","));
  logFile.print(yaccel);
  logFile.print(F(","));
  logFile.print(zaccel);
  logFile.print(F(","));
  logFile.print(xdeg);
  logFile.print(F(","));
  logFile.print(ydeg);
  logFile.print(F(","));
  logFile.print(zdeg);
  logFile.print(F(","));
  logFile.print(xore);
  logFile.print(F(","));
  logFile.print(yore);
  logFile.print(F(","));
  logFile.print(zore);
  logFile.print(F(","));
  logFile.print(Sys);
  logFile.print(F(","));
  logFile.print(Gyro);
  logFile.print(F(","));
  logFile.print(Accel);
  logFile.print(F(","));
  logFile.println(Mag);
  logFile.close();





  // DUPLICATE TEXT FILE: ******************************
  //logFile = SD.open("OVRLRD2.txt", FILE_WRITE);
  logFile = SD.open("datalog2.txt", O_APPEND | O_WRITE);
  logFile.print(GPS.hour);    // Hours (UTC)
  logFile.print(F(","));
  logFile.print(GPS.minute);  // Minutes (UTC)
  logFile.print(F(","));
  logFile.print(GPS.seconds);  // Seconds (UTC)
  logFile.print(F(","));
  logFile.print(input);               // Input (ie command sent via Freeman
  logFile.print(F(","));
  logFile.print(GPS.latitudeDegrees, 7);   // lat
  logFile.print(F(","));
  logFile.print(GPS.longitudeDegrees, 7);  // long
  logFile.print(F(","));
  logFile.print(GPS.altitude);    // alt
  logFile.print(F(","));
  logFile.print(cutdown1);
  logFile.print(F(","));
  logFile.print(cutdown2);
  logFile.print(F(","));
  logFile.print(valve);
  logFile.print(F(","));
  logFile.print(internaltemp);
  logFile.print(F(","));
  logFile.print(internaltemp_lin);
  logFile.print(F(","));
  logFile.print(bnotemp);
  logFile.print(F(","));
  logFile.print(thermistor0_temp);
  logFile.print(F(","));
  logFile.print(pressure);
  logFile.print(F(","));
  logFile.print(xaccel);
  logFile.print(F(","));
  logFile.print(yaccel);
  logFile.print(F(","));
  logFile.print(zaccel);
  logFile.print(F(","));
  logFile.print(xdeg);
  logFile.print(F(","));
  logFile.print(ydeg);
  logFile.print(F(","));
  logFile.print(zdeg);
  logFile.print(F(","));
  logFile.print(xore);
  logFile.print(F(","));
  logFile.print(yore);
  logFile.print(F(","));
  logFile.print(zore);
  logFile.print(F(","));
  logFile.print(Sys);
  logFile.print(F(","));
  logFile.print(Gyro);
  logFile.print(F(","));
  logFile.print(Accel);
  logFile.print(F(","));
  logFile.println(Mag);
  logFile.close();


}
