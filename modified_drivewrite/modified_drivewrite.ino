#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C

SoftwareSerial serial_connection(3, 4); //RX, TX
TinyGPSPlus gps;

File myFile;
int sdPin = 7;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  serial_connection.begin(9600);
  Serial.println("GPS Start");
  gyroSetup();
  magSetup();
  sdSetup();
}

void loop() {
  gps.encode(serial_connection.read());
  if (gps.location.isUpdated()) {
    gpsRead();
    gyroRead();
    newLine();
  }
  //magRead();
}

void gpsRead() {
  double buffer[5] {};
  delay(50);
  while (serial_connection.available()) {
    gps.encode(serial_connection.read());
  }
  gps.encode(serial_connection.read());
  if (gps.location.isUpdated()) {
    buffer[0] = gps.satellites.value();
    buffer[1] = gps.location.lat();
    buffer[2] = gps.location.lng();
    buffer[3] = gps.speed.mph();
    buffer[4] = gps.altitude.feet();

    driveWrite("SAT");
    driveWrite(String(buffer[0]));
    driveWrite(",");

    driveWrite("LAT");
    String temp = String(buffer[1], 4);
    driveWrite(temp);
    driveWrite(",");

    driveWrite("LNG");
    temp = String(buffer[2], 4);
    driveWrite(temp);
    driveWrite(",");

    driveWrite("SPD");
    temp = String(buffer[3], 4);
    driveWrite(String(buffer[3]));
    driveWrite(",");

    driveWrite("ALT");
    temp = String(buffer[4], 4);
    driveWrite(String(buffer[4]));
    driveWrite(",");
    Serial.println("gps written");
    newLine();
  }

}

/*
  String convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  char tmp_str[7]; // temporary variable used in convert function
  sprintf(tmp_str, "%6d", i);
  return String(tmp_str);
  }*/

void gyroSetup() {
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("gyro init complete");

}
void magSetup() {
  Wire.beginTransmission(MAG_ADDR);
  // Select Write register command
  Wire.write(0x60);
  // Set AH = 0x02
  Wire.write(0x02);
  // Set AL = 0xB4, RES for magnetic measurement = 0
  Wire.write(0xB4);
  // Select address register, (0x02 << 2)
  Wire.write(0x08);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(MAG_ADDR, 1);

  // Read status byte
  if (Wire.available() == 1)
  {
    unsigned int c = Wire.read();
  }
  Wire.endTransmission();
  Serial.println("mag init complete");
}


void gyroRead() {
  int accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
  int gyro_x, gyro_y, gyro_z; // variables for gyro raw data
  int buffer[6] {};

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  buffer[0] = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  buffer[1] = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  buffer[2] = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  int16_t temp = Wire.read() << 8 | Wire.read();//pls work
  buffer[3] = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  buffer[4] = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  buffer[5] = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  driveWrite("AX");
  driveWrite(String(buffer[0]));
  driveWrite(",");

  driveWrite("AY");
  driveWrite(String(buffer[1]));
  driveWrite(",");

  driveWrite("AZ");
  driveWrite(String(buffer[2]));
  driveWrite(",");

  driveWrite("GX");
  driveWrite(String(buffer[3]));
  driveWrite(",");

  driveWrite("GY");
  driveWrite(String(buffer[4]));
  driveWrite(",");

  driveWrite("GZ");
  driveWrite(String(buffer[5]));
  driveWrite(",");

  /*  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    int16_t temp = Wire.read() << 8 | Wire.read();//pls work
    gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

    dataString += "AX" + String(accelerometer_x) + ",";
    dataString += "AY" + String(accelerometer_y) + ",";
    dataString += "AZ" + String(accelerometer_z) + ",";
    dataString += "GX" + String(gyro_x) + ",";
    dataString += "GY" + String(gyro_y) + ",";
    dataString += "GZ" + String(gyro_z) + ",";
    return dataString;  */
}

void magRead() {
  unsigned int data[7];

  // Start I2C Transmission
  Wire.beginTransmission(MAG_ADDR);
  // Start single meaurement mode, ZYX enabled
  Wire.write(0x3E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(MAG_ADDR, 1);

  // Read status byte
  if (Wire.available() == 1)
  {
    unsigned int c = Wire.read();
  }
  delay(100);

  // Start I2C Transmission
  Wire.beginTransmission(MAG_ADDR);
  // Send read measurement command, ZYX enabled
  Wire.write(0x4E);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 7 bytes of data
  Wire.requestFrom(MAG_ADDR, 7);

  // Read 7 bytes of data
  // status, xMag msb, xMag lsb, yMag msb, yMag lsb, zMag msb, zMag lsb
  if (Wire.available() == 7);
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
    data[6] = Wire.read();
  }

  // Convert the data
  int xMag = data[1] * 256 + data[2];
  int yMag = data[3] * 256 + data[4];
  int zMag = data[5] * 256 + data[6];

  driveWrite("MX");
  driveWrite(String(xMag));
  driveWrite(",");

  driveWrite("MY");
  driveWrite(String(yMag));
  driveWrite(",");

  driveWrite("MZ");
  driveWrite(String(zMag));
  driveWrite(",");
}

void sdSetup() {
  pinMode(sdPin, OUTPUT);
  if (!SD.begin(sdPin)) {
    Serial.println("SD Failed.");
    while (1);
  }
  Serial.println("SD Success.");

  if (SD.exists("gps-log.txt")) {
    Serial.println("exists. Deleting now.");
    SD.remove("gps-log.txt");
    Serial.println("created");

  }
  else {
    Serial.println("Creating .txt...");
  }
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  myFile.print("Time, Sat Count, Latitude, Longitude, Speed, Altitude\n");
  myFile.close();

  digitalWrite(sdPin, LOW);
}

void driveWrite(String s) {
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(s);
    myFile.close();
  }
}


void newLine() {
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(" ");
    myFile.close();
  }
}
