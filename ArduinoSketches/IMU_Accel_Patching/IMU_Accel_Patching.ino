
#include "MPU9250.h"

#define dXa A2
#define dYa A1
#define dZa A0

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

float roll = 0;
int prevtimeroll = 0;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  Serial.println("We Init");
  analogReadResolution(12);
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

void loop() {
  rollupd();
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(acc(1));
  Serial.print("\t");
  Serial.print(acc(2));
  Serial.print("\t");
  Serial.println(acc(3));
}

//Provide an axis 1-3, 1 is axial, 2 is horizontal, 3 is vertical
float acc(int axis) {
  float low;
  float high;
  IMU.readSensor();
  switch (axis)
  {
    case 1:
      low = IMU.getAccelX_mss()/9.81;
      high = -(analogRead(dYa)*400.0/4095-200);
      break;
    
    case 2:
      low = -IMU.getAccelY_mss()/9.81;
      high = (analogRead(dXa)*400.0/4095-200);
      break;

    case 3:
      low = IMU.getAccelZ_mss()/9.81;
      high = -(analogRead(dZa)*400.0/4095-200);
    default:
      break;
  }
  if (low<-7||low>7)
    return high;
  else
    return low;
      
}

void rollupd (){
  int delT = millis()-prevtimeroll;
  prevtimeroll = millis();
  IMU.readSensor();
  roll += IMU.getGyroX_rads()*57.2957795*delT/1000.0;
}