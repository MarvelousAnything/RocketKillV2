#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <SD.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1020)

const int MPU6050_ADDR = 0x68;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64]; 


Quaternion q;
VectorInt16 aa;

int16_t temperature;

MPU6050 accelgyro;

Adafruit_BMP3XX bmp;

File myFile;

String fname = "a.txt";

void initialize_mpu6050() {
    accelgyro.initialize();

    devStatus = accelgyro.dmpInitialize();

        if (devStatus == 0) {
        
        accelgyro.CalibrateAccel(6);
        accelgyro.CalibrateGyro(6);
        accelgyro.setDMPEnabled(true);

        dmpReady = true;
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    }
}

void setup() {
    if (!SD.begin()) {
        while (1);
    }
    myFile = SD.open(fname, FILE_WRITE);
    if (myFile) {
        myFile.close();
    }

    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    initialize_mpu6050();
}

void loop() {

    if (!dmpReady) return;

    accelgyro.dmpGetQuaternion(&q, fifoBuffer);
    accelgyro.dmpGetAccel(&aa, fifoBuffer);

    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    myFile = SD.open(fname, FILE_WRITE);
    if (myFile.availableForWrite()) {
        myFile.println(String(millis()) + ',' + String(q.w) + ',' + String(q.x) + ',' + String(q.y) + ',' + String(q.z) + ',' + String(aa.x) + ',' + String(aa.y) + ',' + String(aa.z) + ',' + String(alt));
        myFile.close();
    }
    delay(100);
}