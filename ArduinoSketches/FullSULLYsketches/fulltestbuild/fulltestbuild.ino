/*

Team Ski-U-Launch SULLY Rocket On-Board Code
--Kevin Falcetano and Clark Reiter--

This code collects, logs, and transmits rocket telemertry data on the 
Feather M0 LoRa. A set of Adafruit sensor boards a used.

*/

//=================================================================
//      I N I T I A L  D E F I N I T I O N S
//=================================================================
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_BMP3XX.h"
#include <math.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include "MPU9250.h"

#define LED 13
#define STR_LEN 20
//----------------------------------------------------------
// Feather m0 LoRa
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//----------------------------------------------------------
//Inertial Measurment Units
#define dXa A2
#define dYa A1
#define dZa A0
MPU9250 IMU(Wire,0x68);
int status;
float roll = 0;
float velocity = 0;
int prevtimeroll = 0;
int prevtimevel = 0;

//----------------------------------------------------------
//Altimeter
#define SEALEVELPRESSURE_HPA (1013.25)
float initalt = 0;
float barorate = 0;
float prevalt = 0;
int altcount = 0;
float dalt = 0;
int baroT = 0;
Adafruit_BMP3XX bmp; // I2C

//----------------------------------------------------------
//GPS
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false
uint32_t timer = millis();

//----------------------------------------------------------
//SD Logger
#define SD_CS 12
String filename = "1.txt";

//----------------------------------------------------------
const int buffsize = 20;
char buff[buffsize];
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
bool BASE_STATUS_VALID = false;
unsigned long time1 = 0;
int step = 1;

//=================================================================
//      S E T U P
//=================================================================
void setup()
{
  Serial.begin(115200); 
  Serial.println("SULLY Super ALtimeter...");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
//-----------------------------------------------------------------
//LoRa Setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  if (!rf95.init())
    Serial.println("LoRa init failed");

  Serial.println("LoRa radio init OK!...");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("set freq failed");
    while (1);
  }
  Serial.print("Freq set to: "); Serial.println(RF95_FREQ);
  // Set Max Transmit Power
  rf95.setTxPower(23, false);
  // Wait for base station reply to connect boradcast
  while(!ConnectBaseStation()){;}
  
//-----------------------------------------------------------------
//Altimiter Setup
  if (!bmp.begin()) {
    Debug("BMP init failed\n");
    while (1);
  }
  Debug("BMP init success\n");
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  char alt[30];
  float altj = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  snprintf(alt, sizeof(alt),"Junkalt: %f\n",altj);
  Debug(alt);
  delay(1000);
  char alti[30];
  initalt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  snprintf(alti, sizeof(alti),"Initalt: %f\n",initalt);
  Debug(alti);

//----------------------------------------------------------------
//GPS Setup
  Debug("GPS init...\n");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
//----------------------------------------------------------------
//Inertial Measurment Units Setup
  //Manually set analog resolution to its maximum
  analogReadResolution(12);
  status = IMU.begin();
  if (status < 0) {
    Debug("IMU init unsuccessful\n");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
  Debug("IMU init successful\n");
  
//----------------------------------------------------------------
//SD Logger Setup
  digitalWrite(RFM95_CS, HIGH);
  if (!SD.begin(SD_CS)) {
    Debug("Card failed\n");
    // don't do anything more:
    while (1);
  }
  int fnum = 1;
  while(SD.exists(filename)){
    fnum++;
    filename = String(fnum)+".txt";
  }
  Debug("SD card init\n");
  // Additional setup
  Debug("Waiting for GPS fix\n");
  Debug("Press enter to skip\n");
  bool skipgps = false;
  while(!GPS.fix&&!skipgps)
  {
    UpdateGPS();
    if (rf95.available()){
      rf95.recv(buf,&len);
      skipgps = true;
    }
  }
  Debug("Got GPS Fix\n");
  TxGPSData();
  delay(1000);
  Debug("Enter or detect to ");
  Debug("log\n");

}//end setup

//=================================================================
//      M A I N  L O O P
//=================================================================
void loop()
{
  switch (step)
  {
    case 1:
      IMU.readSensor();
      if (acc(1)>6.0||rf95.available()) //Check for message to go ahead or axial acceleration greater than 6G
      {
        rf95.recv(buf,&len);
        step++;
        Debug("Logging data\n");
      }
      break;
    
    case 2:
      UpdateRoll();
      UpdateVelocity();
      LogData();
      if (DetectMain()){
        delay(200);
        Debug("Deploy, transmit\n");
        //rf95.recv(buf,&len);
        step++;        
      }
      break;
    
    case 3:
      SendLoggedData();
      Debug("\nEOF\n");
      step++;
      break;
    
    case 4:
      UpdateGPS();
      if (timer > millis()) timer = millis();
      if (millis() - timer > 2000) {
        timer = millis();
        if(GPS.fix){
          TxGPSData();
        }
        else {
          Debug("No fix\n");
        }
      }
      break;

    default:
      Debug("Step out of range\n");
      delay(2000);
      break;
  }
  
}

//=================================================================
//        U S E R - D E F I N E D  F U N C T I O N S
//=================================================================

void BaseStationInit()
{
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RFM95_CS,LOW);
  uint8_t Tx[] = "Can you hear me base station?";
  rf95.send(Tx,sizeof(Tx));
  rf95.waitPacketSent();
  digitalWrite(RFM95_CS,HIGH);
}

void Debug (char Tx[])
{
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RFM95_CS,LOW);
  Serial.println(Tx);
  rf95.send((uint8_t*)Tx,*Tx);
  rf95.waitPacketSent();
}

void clearbuff() {
  memset(buff, 0, buffsize);
}
 
// converts lat/long from Adafruit
// degree-minute format to decimal-degrees
float DegMinToDecDeg(float degMin) {
  float min = 0.0;
  float decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((float)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}


void SerialPrintGPS(){
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
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(DegMinToDecDeg(GPS.latitude), 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(DegMinToDecDeg(GPS.longitude), 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
}

bool ConnectBaseStation(){
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RFM95_CS,LOW);
  bool BASE_STATUS = false;
  
  if(!BASE_STATUS && millis()- time1 > 5000 )
  {
    BaseStationInit();
    Serial.println("Waiting for Base Station Response...");
    time1 = millis();
  }
  
  if (rf95.available())
  {
    BASE_STATUS = true;
    // Connection Response from Base Station
    if(rf95.recv(buf, &len))
    {
      Serial.println((char*)buf);
      rf95.send(buf,sizeof(buf));
      rf95.waitPacketSent();
    }
    return true;
  }
  else
  {
    return false;
  }
  digitalWrite(RFM95_CS,HIGH);
}

void TxGPSData(){
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RFM95_CS,LOW);
  char lat[STR_LEN],lon[STR_LEN];
  float latf = DegMinToDecDeg(GPS.latitude);
  
  float lonf = DegMinToDecDeg(GPS.longitude);
  
  snprintf(lat, sizeof(lat),"%fN\n",latf);
  snprintf(lon, sizeof(lon),"%fW\n\n",lonf);
  rf95.send((uint8_t*)lat, STR_LEN);
  rf95.waitPacketSent();
  rf95.send((uint8_t*)lon, STR_LEN);
  rf95.waitPacketSent();
}

void LogData(){
  unsigned long inittime = millis();
//Make a big CSV string line to log with timestamps in millis
  IMU.readSensor();
  String dataString = String(millis()/1000.0)+","+String(acc(1))+","+String(acc(2))+","+String(acc(3))+","+String(bmp.readAltitude(SEALEVELPRESSURE_HPA)-initalt)
    +","+String(roll)+","+String(velocity)+","+String(barorate);
//Log the line to SD
  noInterrupts();
  digitalWrite(SD_CS, LOW);
  digitalWrite(RFM95_CS,HIGH);
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile){
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
  else {
    Serial.println("error opening datalog.txt");
  }
  digitalWrite(SD_CS, HIGH);
  digitalWrite(RFM95_CS,LOW);
  interrupts();
}

void SendLoggedData() {
  uint8_t buffer[STR_LEN];
  digitalWrite(SD_CS, LOW);
  digitalWrite(RFM95_CS, HIGH);
  File dataFile = SD.open(filename, FILE_READ);
  if (dataFile){
    while(dataFile.available()){
      for (int i=0;i<STR_LEN&&dataFile.available();i++) {
        buffer[i] = dataFile.read();
      }
      digitalWrite(SD_CS, HIGH);
      digitalWrite(RFM95_CS, LOW);
      rf95.send(buffer,STR_LEN);
      rf95.waitPacketSent();
      digitalWrite(SD_CS, LOW);
      digitalWrite(RFM95_CS, HIGH);
    }
  }
  else
  {
    Debug("No log file\n");
  }
}

float acc(int axis) {
  //Call IMU.readSensor() before using this
  float low;
  float high;
  switch (axis)
  {
    case 1:
      low = -IMU.getAccelX_mss()/9.81;
      high = (analogRead(dYa)*400.0/4095-200);
      break;
    
    case 2:
      low = IMU.getAccelY_mss()/9.81;
      high = -(analogRead(dXa)*400.0/4095-200);
      break;

    case 3:
      low = -IMU.getAccelZ_mss()/9.81;
      high = (analogRead(dZa)*400.0/4095-200);
    default:
      break;
  }
  if (low<-7||low>7)
    return high;
  else
    return low;
      
}

void UpdateGPS()
{
 char c = GPS.read();
 if (GPS.newNMEAreceived()) {
    GPS.lastNMEA(); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void UpdateRoll (){
  int delT = millis()-prevtimeroll;
  prevtimeroll = millis();
  IMU.readSensor();
  roll += IMU.getGyroX_rads()*57.2957795*delT/1000.0;
}
/*
void UpdateVelocity (){
  int delT = millis()-prevtimevel;
  prevtimevel = millis();
  IMU.readSensor();
  velocity += (sqrtf(acc(1)*acc(1)+acc(2)*acc(2)+acc(3)*acc(3))*9.81-9.81)*delT/1000;
}*/
void UpdateVelocity (){
  int delT = millis()-prevtimevel;
  prevtimevel = millis();
  IMU.readSensor();
  velocity += (sqrtf(acc(1)*acc(1)+acc(2)*acc(2)+acc(3)*acc(3))*9.81-9.81)*delT/1000;
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA)-initalt;
  dalt += (alt-prevalt)/(delT/1000.0);
  altcount++;
  baroT += delT;
  if (baroT>100){
    barorate = dalt/altcount;
    dalt = 0;
    baroT = 0;
    altcount = 0;
  }
  prevalt = alt;
}

bool DetectMain() {
  if(barorate<-5.0&&(bmp.readAltitude(SEALEVELPRESSURE_HPA)-initalt)<182.9) {
    return true;
  }
  else {
    return false;
  }
  //Detect descent faster than 13 ft/s and altitude less than 600ft
}