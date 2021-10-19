#include <SPI.h>
#include <SD.h>

File myFile;

void setup() {
    Serial.begin(9600);
    pinMode(10, OUTPUT);
    
    if(!SD.begin(10)){
        Serial.println("SD Init Failed...");
        while(1);
    }
    Serial.println("SD Init Success...");
    
    //probably gonna want to wipe the SD card and start new each time. Or add a time and date.
    if (SD.exists("gps-log.txt")) {
        Serial.println("gps-log exists.");
        }     
    else {
            Serial.println("example.txt doesn't exist. Creating Now...");
                myFile = SD.open("gps-log.txt", FILE_WRITE);
                myFile.close();
        }
    

}

void loop() {
    for(int i = 0; i < 10; i++){
        driveWrite((String)i);
    }
    
    delayText();    
    
    driveRead();
    Serial.println("finished. Clear txt? y / n");
    
    while(!Serial.available());
    
    char selection = Serial.read();
    
    switch(selection){
        case 'y':
        SD.remove("gps-log.txt");
        Serial.println("removed");
        break;

        case 'n':
            Serial.println("saving data");
            break;
            
        default:
            Serial.println("Removing");
            SD.remove("gps-log.txt");
    }

    Serial.println("Run write example again? y / n");

    while(!Serial.available());

    selection = Serial.read();

    switch(selection){
	case 'y':
		break;
	case 'n':
		Serial.println("exiting example...");
		while(1);
	default:
		Serial.println("running again...");
		break;
   	 }

}

void driveRead(){
    myFile = SD.open("gps-log.txt");
    if(myFile){
        Serial.println("GPS Log: ");

        while(myFile.available()){
        Serial.write(myFile.read());
        }
        myFile.close();
    }
}

void driveWrite(String s){
    myFile = SD.open("gps-log.txt", FILE_WRITE);
        if(myFile){
                myFile.println(s);
        }
        myFile.close();
}

void delayText(){
    Serial.println("write complete");
    Serial.println("Preparing to Read...");
    Serial.print("3");
    for(uint8_t i = 0; i < 4; i++){
            Serial.print(".");
            delay(250);
    }
    
    Serial.print("2");
    for(uint8_t i = 0; i < 4; i++){
            Serial.print(".");
            delay(250);
    }
    Serial.print("1");
    for(uint8_t i = 0; i < 4; i++){
            Serial.print(".");
            delay(250);
    }
    Serial.println();
}
