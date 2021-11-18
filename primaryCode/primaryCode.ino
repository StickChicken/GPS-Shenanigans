#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "printf.h"
#include "RF24.h"

#define MPU_ADDR 0x68
#define MAG_ADDR 0x0C


//radio constants
RF24 radio(9, 10); // Use pin 9 for radio CE, use pin 10 for radio CSN
uint8_t address[][6] = {"1Node", "2Node"}; //addresses for paths between radios

SoftwareSerial serial_connection(3, 4); //RX, TX
TinyGPSPlus gps;

File myFile;
byte sdPin = 7;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  serial_connection.begin(9600);
  Serial.println("GPS Start");
  gyroSetup();
  magSetup();
  sdSetup();
  //radioSetup();
}

void loop() {
  char buf[100];
  //String data = gpsRead();
  String gyrodata = gyroRead();
  int temp[3];
  magRead(temp);
  String magData = String(temp[0]) + "," + String(temp[1]) + "," + String(temp[2]);
  //driveWrite(data);
  Serial.println(data);
  delay(500);
}

String gpsRead() {
  String toReturn = "";
  while (serial_connection.available()) {
    gps.encode(serial_connection.read());
  }
  gps.encode(serial_connection.read());
  if (gps.location.isUpdated()) {
    toReturn += "SAT" + String(gps.satellites.value()) + ",";
    toReturn += "LAT" + String(gps.location.lat(), 6) + ",";
    toReturn += "LNG" + String(gps.location.lng(), 6) + ",";
    toReturn += "SPD" + String(gps.speed.mph(), 2) + ",";
    toReturn += "ALT" + String(gps.altitude.feet(), 2) + ",";
  }
  return toReturn;
}



void radioWrite(char msg[]) {
  radio.flush_tx();
  radioSend(msg);
}

bool radioSend(char msg[]) {
  bool report = 0;
  char toWrite[32];
  if (strlen(msg) <= 32) {
    strncpy(toWrite, msg, 32);
    report = radio.write(&toWrite, 32);
  } else {
    for (int i = 0; i < strlen(msg); i += 32) {
      strncpy(toWrite, msg + i, 32);
      report = radio.write(&toWrite, strlen(toWrite));
    }
  }
  return report;
}

String convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  char tmp_str[7]; // temporary variable used in convert function
  sprintf(tmp_str, "%6d", i);
  return String(tmp_str);
}

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


String gyroRead() {
  int accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
  int gyro_x, gyro_y, gyro_z; // variables for gyro raw data
  String dataString = "";
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7 * 2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
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
  return dataString;
}

void magRead(int temp[]) {
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
  temp[0] = data[1] * 256 + data[2];
  temp[1] = data[3] * 256 + data[4];
  temp[2] = data[5] * 256 + data[6];
}

void radioSetup() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(32); // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[0]);     // always uses pipe 0
  radio.stopListening();  // put radio in TX mode
  Serial.println("Radio init success...");
}

void sdSetup() {
  pinMode(sdPin, OUTPUT);
  if (!SD.begin(sdPin)) {
    Serial.println("SD Init Failed...");

  }
  Serial.println("SD Init Success...");

  if (SD.exists("gps-log.txt")) {
    Serial.println("gps-log exists. Deleting now...");
    SD.remove("gps-log.txt");
    Serial.println("gps-log created.");

  }
  else {
    Serial.println("example.txt doesn't exist. Creating Now...");
  }
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  myFile.print("Time, Sat Count, Latitude, Longitude, Speed, Altitude\n");
  myFile.close();

  digitalWrite(sdPin, LOW);
}

void driveWrite(String s) {
  digitalWrite(sdPin, HIGH);
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(s);
  }
  myFile.close();
  digitalWrite(sdPin, LOW);
}
