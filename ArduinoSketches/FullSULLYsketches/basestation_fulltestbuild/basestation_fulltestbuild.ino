// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

//for arduino uno
#define RFM95_INT     3  // `
#define RFM95_CS      4  //
#define RFM95_RST     2  // "A"
#define LED      

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
  Serial.flush();
  while (!Serial){;}

  Serial.println("Arduino Uno Basestation...");

  if (!rf95.init())
    Serial.println("init failed");
  
  Serial.println("Base Station LoRa Radio init Success!");

  
  if (!rf95.setFrequency(RF95_FREQ))
    Serial.println("setFrequency failed");


  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  
  Serial.println("Connecting to Rocket...");
  
}


//int16_t packetnum = 0;  // packet counter, we increment per xmission
bool CON_STATUS = false; // rocket connection status
unsigned long rxtime = 0;
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t TX1[] = "Base Station: Connected\n";

void loop()
{
//RadioRoutine();
if (Serial.available())
{
  char inChar = (char)Serial.read();
  Serial.println("--ping--");
  uint8_t Msg[] = "S";
  rf95.send(Msg,sizeof(Msg));
}

  if (rf95.available())
  { 
    
    if (!CON_STATUS)
    {
      
        if(rf95.recv(buf, &len))
        {
          
          Serial.println("Connected to rocket");
          Serial.println((char*)buf);
          CON_STATUS = true;
          rxtime = millis();
          //Serial.println("attempting tx");
          rf95.send(TX1,sizeof(TX1));
          rf95.waitPacketSent();
        }

       
     } //end if(!CON_STATUS)

    if(rf95.recv(buf, &len))
    {
      //Serial.println("Normal Stream");
      Serial.print((char*)buf);
      clearbuff();
    }


   }

  
   
} //end void loop()


void clearbuff() {
  memset(buf, 0, RH_RF95_MAX_MESSAGE_LEN);
}
