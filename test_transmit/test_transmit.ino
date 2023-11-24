#include <Arduino.h>
#include <TinyMPU6050.h>
#include <SPI.h>
#include <BMP280_DEV.h> 
#include <Wire.h>


float temp, press, alt;  
BMP280_DEV bmp280;  
MPU6050 mpu (Wire);

void setup() {
  // put your setup code here, to run once:
  // Setting mpu
  mpu.Initialize(); 
  Serial.begin(9600);
  Wire.begin();
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
  Serial.println("Offsets:");
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());

  // Setting bmp

  bmp280.begin(BMP280_I2C_ALT_ADDR);
  bmp280.setTimeStandby(TIME_STANDBY_2000MS);     // Set the standby time to 2 seconds
  bmp280.startNormalConversion();   
  

}

void loop() {
  mpu.Execute();
  
  roll = mpu.GetAngX();
  pitch = mpu.GetAngY();
  yaw = mpu.GetAngZ();
  float a_x = mpu.GetAccX();
  float a_y = mpu.GetAccY();
  float a_z = mpu.GetAccZ();
  bmp280.getMeasurements(temp, press, alt);

  String data = "Temperature:" + String(temp) + "," +
               "Pressure:" + String(press) + "," +
               "Altitude:" + String(alt)+"," +
               "AccelX:" + String(a_x) + "," +
               "AccelY:" + String(a_y) + "," +
               "AccelZ:" + String(a_z) + "," +
               "GyroX:" + String(roll) + "," +
               "GyroY:" + String(pitch) + "," +
               "GyroZ:" + String(yaw);

  Serial.println(data);
  

}
