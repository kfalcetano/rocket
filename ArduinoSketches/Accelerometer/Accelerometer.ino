#define LED 13
#define xacc A0
#define yacc A1
#define zacc A2

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  delay(100);
  //Manually set analog resolution to its maximum
  analogReadResolution(12);
}

void loop() {
  //Plot all of the channels to test (may need to invert for the proper values)
  //X-axis will be the vertical axis, but it may be positive or negative depending on 
  Serial.print(getGs(analogRead(xacc)));
  Serial.print("\t");
  Serial.print(getGs(analogRead(yacc)));
  Serial.print("\t");
  Serial.println(getGs(analogRead(zacc)));
}

// Function to convert 12 bit digital values to G values
float getGs(int dig)
{
  //assumes zero volts is being both output and sensed correctly (it isn't)
  return (dig-2048)/2048.0*200.0;
}



