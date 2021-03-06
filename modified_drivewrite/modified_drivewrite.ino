#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <RF24.h>

#define MPU_ADDR 0x68
//#define MAG_ADDR 0x0C

//Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

SoftwareSerial serial_connection(3, 4); //RX, TX
TinyGPSPlus gps;
RF24 radio(9, 10);

uint8_t address[][6] = {"1Node", "2Node"}; //addresses for paths between radios

File myFile;
int sdPin = 7;

bool gpsCheck = false;
int count = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  serial_connection.begin(9600);
  Serial.println("GPS Start");
  gyroSetup();
  sdSetup();
  radioSetup(); 

}

void loop() {
  gpsRead();
  if (gpsCheck) {    
    gyroRead();
    newLine();
    Serial.println("data sent");
  }
  delay(100);
}

void gpsRead() {
  gpsCheck = false;
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

    driveWrite(String(buffer[0]));
    driveWrite(",");
    radioSend(String(buffer[0]) + ",");
    
    String temp = String(buffer[1], 4);
    driveWrite(temp);
    driveWrite(",");
    radioSend(temp + ",");

    temp = String(buffer[2], 4);
    driveWrite(temp);
    driveWrite(",");
    radioSend(temp + ",");
    
    temp = String(buffer[3], 4);
    driveWrite(String(buffer[3]));
    driveWrite(",");
    radioSend(temp + ",");

    temp = String(buffer[4], 4);
    driveWrite(String(buffer[4]));
    driveWrite(",");
    radioSend(temp + ",");

    gpsCheck = true;
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

/*
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
*/


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

  driveWrite(String(buffer[0]));
  driveWrite(",");

  radioSend("AX " + String(buffer[0]));

  delay(10);

  driveWrite(String(buffer[1]));
  driveWrite(",");
  radioSend("AY " + String(buffer[1]));

  delay(10);

  driveWrite(String(buffer[2]));
  driveWrite(",");
  radioSend("AZ " + String(buffer[2]));

  delay(10);

  driveWrite(String(buffer[3]));
  driveWrite(",");
  radioSend("GX " + String(buffer[3]));

  delay(10);

  driveWrite(String(buffer[4]));
  driveWrite(",");
  radioSend("GY " + String(buffer[4]));

  delay(10);

  driveWrite(String(buffer[5]));
  driveWrite(",");
  radioSend("GZ " + String(buffer[5]));
  /*
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
  */
}

/*
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
*/
/*
  void getBNO(){

  sensors_event_t angVelocityData , magnetometerData, accelerometerData;
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);


  printEvent(&accelerometerData);
  printEvent(&angVelocityData);
  printEvent(&magnetometerData);
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  delay(100);
  }

  void printEvent(sensors_event_t* event) {
  float x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  String temp = "";
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
    driveWrite("AX");
    temp = String(x,4);
    driveWrite(temp);
    driveWrite(",");

    driveWrite("AY");
    temp = String(y,4);
    driveWrite(temp);
    driveWrite(",");

    driveWrite("AZ");
    temp = String(z,4);
    driveWrite(temp);
    driveWrite(",");
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;

    driveWrite("MX");
    driveWrite(String(x, 4));
    driveWrite(",");

    driveWrite("MY");
    driveWrite(String(y,4));
    driveWrite(",");

    driveWrite("MZ");
    driveWrite(String(z,4));
    driveWrite(",");
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;

    driveWrite("GX");
    driveWrite(String(x, 4));
    driveWrite(",");

    driveWrite("GY");
    driveWrite(String(y,4));
    driveWrite(",");

    driveWrite("GZ");
    driveWrite(String(z,4));
    driveWrite(",");
  }
  }
*/

void radioSend(String str) {
  int len = str.length();
  char msg[len];
  str.toCharArray(msg,str.length());
  char toWrite[32];
  if (strlen(msg) <= 32) {
    strncpy(toWrite, msg, 32);
    radio.write(&toWrite, 32);
  } else {
    for (int i = 0; i < strlen(msg); i += 32) {
      strncpy(toWrite, msg + i, 32);
      radio.write(&toWrite, strlen(toWrite));
    }
  }
}

void radioSetup(){
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

  // additional setup specific to the node's role
  radio.stopListening();  // put radio in TX mode
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
  myFile.print("Sat Count, Latitude, Longitude, Speed, Altitude, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z\n");
  myFile.close();
}

void driveWrite(String s) {
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(s);
    myFile.close();
  }
}

void driveRead() {
  myFile = SD.open("gps-log.txt");
  if (myFile) {
    Serial.println("gps-log.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}

void newLine() {
  myFile = SD.open("gps-log.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("");
    myFile.close();
  }
}
