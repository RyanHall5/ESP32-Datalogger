/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define port 80

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

const char *ssid_Router      = "ryan_iphone";  //input your wifi name
const char *password_Router  = "sunshine";  //input your wifi passwords
WiFiServer  server(port);

Adafruit_BMP280 bmp; // I2C

const int MPU = 0x68; // MPU6050 I2C address

int timedelay = 100;
int sampleAmount = 1000;
int16_t rawGX, rawGY, rawGZ;
int16_t rawAX, rawAY, rawAZ;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
float temperature, altitude, pressure;
float local_hPa = 997.63; //~1000 around RIT
long dataCount = 0;
int c = 0;

void setup(){

  Serial.begin(115200);

  connectToWifi();

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); 

  calculate_IMU_error();  

  //bmp stuff
  bmp.begin(0x76);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}
void loop(){

  WiFiClient client = server.available();

  if (client) {
    Serial.println("Client connected!");
    unsigned long lastSend = millis();

    //String header = "packetNumber,timestamp,rssi,roll,pitch,yaw,altitude,pressure,temperature";

    //client.println(header);
    while (client.connected()) {
      

      //getting data values
      String rssi = String(WiFi.RSSI());
      updateRPY();
      updateBMPData();

      String countAsString = String(dataCount);

      String data = "b" + countAsString + "," + rssi + "," + roll + "," + pitch + "," + yaw + "," + altitude + "," + pressure + "," + temperature;
      client.println(data + "\n");
      Serial.println("Sent to laptop: " + data);
      dataCount += 1;
      delay(50);
    }

    client.stop();
    Serial.println("Client disconnected.");
  }

}

void updateBMPData(){
  altitude = bmp.readAltitude(local_hPa); //1006.8 at RIT
  pressure = bmp.readPressure();
  temperature = bmp.readTemperature();
}

void connectToWifi()
{
    Serial.printf("\nConnecting to ");
    Serial.println(ssid_Router);
    WiFi.disconnect();
    WiFi.begin(ssid_Router, password_Router);
    delay(1000);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());			
    Serial.printf("IP port: %d\n",port);			
    server.begin(port);								
    WiFi.setAutoReconnect(true);
}
void updateRPY(){
   
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  rawAX = (int16_t)((Wire.read() << 8) | Wire.read());
  AccX = rawAX / 16384.0; // deg/s for FS = ±250 dps
  rawAY = (int16_t)((Wire.read() << 8) | Wire.read());
  AccY = rawAY / 16384.0; // deg/s for FS = ±250 dps
  rawAZ = (int16_t)((Wire.read() << 8) | Wire.read());
  AccZ = rawAZ / 16384.0; // deg/s for FS = ±250 dps

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read

  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers


  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  rawGX = (int16_t)((Wire.read() << 8) | Wire.read());
  GyroX = rawGX / 131.0; // deg/s for FS = ±250 dps
  rawGY = (int16_t)((Wire.read() << 8) | Wire.read());
  GyroY = rawGY / 131.0; // deg/s for FS = ±250 dps
  rawGZ = (int16_t)((Wire.read() << 8) | Wire.read());
  GyroZ = rawGZ / 131.0; // deg/s for FS = ±250 dps

  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  //getting elapsed time
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values sampleAmount times
  while (c < sampleAmount) {
      Wire.beginTransmission(MPU);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      rawAX = (int16_t)((Wire.read() << 8) | Wire.read());
      AccX = rawAX / 16384.0; // deg/s for FS = ±250 dps
      rawAY = (int16_t)((Wire.read() << 8) | Wire.read());
      AccY = rawAY / 16384.0; // deg/s for FS = ±250 dps
      rawAZ = (int16_t)((Wire.read() << 8) | Wire.read());
      AccZ = rawAZ / 16384.0;
      // Sum all readings
      AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
      AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
      c++;
  }
  //Divide the sum by sampleAmount to get the error value
  AccErrorX = AccErrorX / sampleAmount;
  AccErrorY = AccErrorY / sampleAmount;
  c = 0;
  // Read gyro values sampleAmount times
  while (c < sampleAmount) {
      Wire.beginTransmission(MPU);
      Wire.write(0x43);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU, 6, true);
      rawGX = (int16_t)((Wire.read() << 8) | Wire.read());
      rawGY = (int16_t)((Wire.read() << 8) | Wire.read());
      rawGZ = (int16_t)((Wire.read() << 8) | Wire.read());
      // Sum all readings
      GyroErrorX = GyroErrorX + (rawGX / 131.0);
      GyroErrorY = GyroErrorY + (rawGY / 131.0);
      GyroErrorZ = GyroErrorZ + (rawGZ / 131.0);
      c++;
  }
  //Divide the sum by sampleAmount to get the error value
  GyroErrorX = GyroErrorX / sampleAmount;
  GyroErrorY = GyroErrorY / sampleAmount;
  GyroErrorZ = GyroErrorZ / sampleAmount;
}