#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "Adafruit_BMP3XX.h"
#include <math.h>
#define LED 13
//----------------------------------------------------------
// Feather m0 LoRa
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
//----------------------------------------------------------
//Accelerometer
#define xacc A0
#define yacc A1
#define zacc A2
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//----------------------------------------------------------
//Altimeter
#define SEALEVELPRESSURE_HPA (1013.25)
float initalt=0.0;
Adafruit_BMP3XX bmp; // I2C

void ErrCode(int flash);
void SendAltData();
void clearbufc();
void SendGPSPos();

const int bufcsize = 20;
char bufc[bufcsize];
//----------------------------------------------------------
//GPS Setup

#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


void setup()
{
  Serial.begin(115200);
  Serial.println("Boot Sequence Start");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  //----------------------------------------------------------------------------------------------------
//Altimiter
  if (!bmp.begin()) {
    ErrCode(3);
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //----------------------------------------------------------------------------------------------------
//LoRa Setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  Serial.println("Feather LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    ErrCode(5);
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    ErrCode(4);
    while (1);
  }
//----------------------------------------------------------------------------------------------------
//Accelerometer
  
  delay(100);
  //Manually set analog resolution to its maximum
  analogReadResolution(12);
  
  //----------------------------------------------------------------------------------------------------
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
 
//----------------------------------------------------------------------------------------------------
  //GPS Setup
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

}

void loop()
{
  //Call out to the basestation
  delay(5000);
  ErrCode(1);
  ShoutIntoTheVoid("Hello");
  Serial.println("Waiting for response");
  if (rf95.available())
  {

    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    bool escape=false;
    ErrCode(2);
//Main routine for checking we have connection and beginning data stream
//Make sure we recieve data, it's the right message, and we're not trying to end
    if(rf95.recv(buf, &len))
    {
      sprintf(bufc,(char*)buf);
      Serial.println(bufc);
    }
    if(bufc[0]=='b')
    {
      while (!escape)
      {
         // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
        
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
          delay(2000);
          Serial.println("Got a fix");
          Serial.println(GPS.latitude, 4);
          Serial.println(GPS.longitude, 4);
          ShoutIntoTheVoid("Got me a fix");
          SendGPSPos();
        }
        else
        {
           ShoutIntoTheVoid("Waiting for Fix");
        }
        
      }
    }
    escape=false;
   }
   clearbufc();

}

void SendAltData(){
  sprintf(bufc,"%1.3f",bmp.readAltitude(SEALEVELPRESSURE_HPA));
  rf95.send((uint8_t*)bufc,bufcsize);
  rf95.waitPacketSent();
  clearbufc();
}

void SendGPSPos(){
  sprintf(bufc,"Latitude: %f",GPS.latitude);
  rf95.send((uint8_t*)bufc,bufcsize);
  rf95.waitPacketSent();
  clearbufc();
  sprintf(bufc,"Longitude %f",-1.0*GPS.longitude);
  rf95.send((uint8_t*)bufc,bufcsize);
  rf95.waitPacketSent();
  clearbufc();
}

void ShoutIntoTheVoid(char* msg)
{
  sprintf(bufc,msg);
  rf95.send((uint8_t*)bufc,bufcsize);
  rf95.waitPacketSent();
  clearbufc();
}

void ErrCode(int flash) {
   for (int i=0;i<flash;i++)
    {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(100);
    }
    delay(1000);
}

void clearbufc() {
  memset(bufc, 0, bufcsize);
}
