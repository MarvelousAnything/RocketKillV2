#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
// #include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SD.h>
// #include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps20.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1020)

const int MPU6050_ADDR = 0x68;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

int c = 0;

int16_t temperature;

Adafruit_BMP3XX bmp;

File myFile;

String fname = "data.txt";

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void setup() {
    pinMode(6, OUTPUT);

    if (!bmp.begin_I2C()) {
        digitalWrite(6, LOW);
        while(1);
    }

    pinMode(10, OUTPUT);
    if (!SD.begin(10)) {
        digitalWrite(6, LOW);
        while (1);
    }
    digitalWrite(6, HIGH);
    
    myFile = SD.open(fname, FILE_WRITE);
    if (myFile) {
        myFile.println("time,alt,ax,ay,az,gx,gy,gz,rl,pt,yw");
        myFile.close();
    }

    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    calculate_IMU_error();
    delay(1000);
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);
    digitalWrite(6, LOW);
}

void loop() {

    // === Read acceleromter data === //
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)
    // === Read gyroscope data === //
    previousTime = currentTime;        // Previous time is stored before the actual time read
    currentTime = millis();            // Current time actual time read
    elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    // Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
    GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
    GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY = gyroAngleY + GyroY * elapsedTime;
    yaw =  yaw + GyroZ * elapsedTime;
    // Complementary filter - combine acceleromter and gyro angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    if (gyroAngleX == NAN) {
        gyroAngleX = 0.0;
    }

    if (gyroAngleY == NAN) {
        gyroAngleY = 0.0;
    }

    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print("acc: ");
    Serial.print(AccX);
    Serial.print(", ");
    Serial.print(AccY);
    Serial.print(", ");
    Serial.println(AccZ);

    if (AccZ != 0.0 && millis() % 2 == 0) {
        digitalWrite(6, HIGH);
        delay(50);
        digitalWrite(6, LOW);
    }

    Serial.print("gyro: ");
    Serial.print(GyroX);
    Serial.print(", ");
    Serial.print(GyroY);
    Serial.print(", ");
    Serial.println(GyroZ);

    Serial.print("r/p/y: ");
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.print(yaw);
    Serial.println(")");

    myFile = SD.open(fname, FILE_WRITE);
    if (myFile.availableForWrite()) {
        // myFile.println(String(millis()) + ',' + String(alt) + ',' + String(AccX) + ',' + String(AccY) + ',' + String(AccZ)
        // + ',' + String(GyroX) + ',' + String(GyroY) + ',' + String(GyroZ)
        // + ',' + String(roll) + ',' + String(pitch) + ',' + String(yaw));
        myFile.print(millis());
        myFile.print(',');
        myFile.print(alt);
        myFile.print(',');
        myFile.print(AccX);
        myFile.print(',');
        myFile.print(AccY);
        myFile.print(',');
        myFile.print(AccZ);
        myFile.print(',');
        myFile.print(GyroX);
        myFile.print(',');
        myFile.print(GyroY);
        myFile.print(',');
        myFile.print(GyroZ);
        myFile.print(',');
        myFile.print(roll);
        myFile.print(',');
        myFile.print(pitch);
        myFile.print(',');
        myFile.println(yaw);
        myFile.close();
    }
    delay(100);
}