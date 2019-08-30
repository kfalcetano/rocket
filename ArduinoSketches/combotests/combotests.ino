#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP3XX.h"
#include <math.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#define LED 13
#define xacc A0
#define yacc A1
#define zacc A2
float initalt=0.0;

Adafruit_BMP3XX bmp; // I2C

void setup() {
  //----------------------------------------------------------
  //Accelerometer
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  delay(100);
  //Manually set analog resolution to its maximum
  analogReadResolution(12);
  
  //----------------------------------------------------------
  //Altimiter
  //hold until serial
  while (!Serial);
  Serial.println("BMP388 test");

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(1000);
  bmp.readAltitude(SEALEVELPRESSURE_HPA);
  delay(100);
  initalt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
}
  //-----------------------------------------------------------
void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA)-initalt);
  Serial.print(",");
  Serial.println(sqrt(pow(getGs(analogRead(xacc)),2)+pow(getGs(analogRead(yacc)),2)+pow(getGs(analogRead(zacc)),2)));
  delay(5);
}

float getGs(int dig)
{
  //assumes zero volts is being both output and sensed correctly (it isn't)
  return (dig-2048)/2048.0*200.0;
}
