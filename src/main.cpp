#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <SD.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1020)

const int MPU6050_ADDR = 0x68;


//int16_t acc_x, acc_y, acc_z;
//int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

MPU6050 accelgyro;

Adafruit_BMP3XX bmp;

File myFile;

void initialize_mpu6050() {
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
    /*A tidbit on how PID (PI actually) tuning works.
      When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and
      integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral
      uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it
      to the integral value. Each reading narrows the error down to the desired offset. The greater the error from
      set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the
      integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the
      noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100
      readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to
      the fact it reacts to any noise.
    */
    accelgyro.CalibrateAccel(6);
    accelgyro.CalibrateGyro(6);
    Serial.println("\nat 600 Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("700 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("800 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("900 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println();
    accelgyro.CalibrateAccel(1);
    accelgyro.CalibrateGyro(1);
    Serial.println("1000 Total Readings");
    accelgyro.PrintActiveOffsets();
    Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:");
} // Initialize

void setup() {
    Serial.begin(9600);

    if (!bmp.begin_I2C()) {
        Serial.println("I2C FAILURE BMP");
        while (1);
    }

    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    Serial.println("Initializing SD card...");
    if (!SD.begin()) {
        Serial.println("Initialization failed!");
        while (1);
    }

    Serial.println("Initialization done.");
    SD.remove("data.txt");
    myFile = SD.open("data.txt", FILE_WRITE);
    if (myFile) {
        myFile.println("time,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,alt");
        myFile.close();
    } else {
        Serial.println("Error opening data.txt");
    }
}

void loop() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 7*2, true);

    float acc_x = Wire.read()<<8 | Wire.read();
    float acc_y = Wire.read()<<8 | Wire.read();
    float acc_z = Wire.read()<<8 | Wire.read();

    float gyro_x = Wire.read()<<8 | Wire.read();
    float gyro_y = Wire.read()<<8 | Wire.read();
    float gyro_z = Wire.read()<<8 | Wire.read();

    float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA); //METERS

    Serial.println("Accelerometer: (" + String(acc_x) + ", " + String(acc_y) + ", " + String(acc_z) + ") | Altitude: " + String(alt) + "m | Gyro: (" + String(gyro_x) + ", " + String(gyro_y) + ", " + String(gyro_z) + ")");

    myFile = SD.open("data.txt", FILE_WRITE);
    if (myFile.availableForWrite()) {
        myFile.println(String(millis()) + "," + String(acc_x) + "," + String(acc_y) + "," + String(acc_z) + "," + String(gyro_x) + "," + String(gyro_y) + "," + String(gyro_z) + "," + String(alt));
        myFile.close();
    }
    delay(100);
}