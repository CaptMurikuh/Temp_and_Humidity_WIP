#include <i2c_t3.h>

/*************************************************** 
  This is an example for the AM2315 Humidity + Temp sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/1293

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
/*
 Connect RED of the AM2315 sensor to 5.0V
 Connect BLACK to Ground
 Connect WHITE to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
 Connect YELLOW to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
 
 Pin 5 is SD card detect (detects when card is pushed in and put out.
 Pin 6 is Beeper
 Pin 7 is Encoder Button
 Pin 8 is Stop Button
 Pin 25 is Hot-Side Lamp Relay
 Pin 26 is Cold-Side Lamp Relay
 Pin 27 is Mister Relay
 Pin 28 is Misc. Relay

The circuit:
 SD card attached to SPI bus as follows:
 MOSI - pin 11
 MISO - pin 12
 CLK - pin 13
 CS - pin 4
 28, 27, 26, 25
*/


#define AM2315_I2CADDR       0x5C
#define AM2315_READREG       0x03

int hotSideTemp,coldSideTemp;  //Hot-Side and Cold-Side, respectively         //temp, temp1
int hotSideHumidity,coldSideHumidity;  //Hot-Side and Cold-Side, respectively //humidity, humidity1

const int hotSideLampPin = 25;  
int hotSideLampState = LOW;    
const int coldSideLampPin = 26; 
int coldSideLampState = LOW;   
const int misterPin = 27;       
int misterState = LOW;         
const int miscPin = 28;
int miscState = LOW;

long previousMillis = 0;
unsigned long currentMillis = millis();


//Displaying the Data to Serial Monitor
void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire1.begin();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //if(!SD.begin(4)) {
   // Serial.println("SD Card Initialization Failure.");
    //return;
  //}
  //dataLog = SD.open("dataLog.txt", FILE_WRITE);
  
  
  pinMode(hotSideLampPin, OUTPUT);
  pinMode(coldSideLampPin, OUTPUT);
  pinMode(misterPin, OUTPUT);
  pinMode(miscPin, OUTPUT);
  Serial.println("AM2315 Test!");

  digitalWrite(hotSideLampPin, HIGH);
  delay(500);
  digitalWrite(coldSideLampPin, HIGH);
  delay(500);
  digitalWrite(misterPin, HIGH);
  delay(500);
  digitalWrite(miscPin, HIGH);
  delay(500);
  digitalWrite(hotSideLampPin, LOW);
  delay(500);
  digitalWrite(coldSideLampPin, LOW);
  delay(500);
  digitalWrite(misterPin, LOW);
  delay(500);
  digitalWrite(miscPin, LOW);
  delay(1000);
}



                                 //Function to read data from Sensor #1
boolean hotSideSensor() {
   uint8_t reply[10];
  
  // Wake up the sensor
  Wire.beginTransmission(AM2315_I2CADDR);
  delay(2);
  Wire.endTransmission();

  // OK we're ready!
  Wire.beginTransmission(AM2315_I2CADDR);
  Wire.write(AM2315_READREG);
  Wire.write(0x00);  // start at address 0x0
  Wire.write(4);  // request 4 bytes data
  Wire.endTransmission();
  
  delay(10); // add delay between request and actual read!

  Wire.requestFrom(AM2315_I2CADDR, 8);
  for (uint8_t i=0; i<8; i++) {
    reply[i] = Wire.read();
    //Serial.println(reply[i], HEX);
  }
  
  if (reply[0] != AM2315_READREG) return false;
  if (reply[1] != 4) return false; // bytes req'd
  
  hotSideHumidity = reply[2];
  hotSideHumidity *= 256;
  hotSideHumidity += reply[3];
  hotSideHumidity /= 10;
  //Serial.print("H"); Serial.println(humidity);

  hotSideTemp = reply[4] & 0x7F;
  hotSideTemp *= 256;
  hotSideTemp += reply[5];
  hotSideTemp /= 10;
  hotSideTemp = (hotSideTemp * 1.8) + 32;
  //Serial.print("T"); Serial.println(temp);

  // change sign
  if (reply[4] >> 7) hotSideTemp = -hotSideTemp;

  return true;
}





                                 //Function to read data from Sensor #2
boolean coldSideSensor() {
   uint8_t reply[10];
  
  // Wake up the sensor
  Wire1.beginTransmission(AM2315_I2CADDR);
  delay(2);
  Wire1.endTransmission();

  // OK we're ready!
  Wire1.beginTransmission(AM2315_I2CADDR);
  Wire1.write(AM2315_READREG);
  Wire1.write(0x00);  // start at address 0x0
  Wire1.write(4);  // request 4 bytes data
  Wire1.endTransmission();
  
  delay(10); // add delay between request and actual read!

  Wire1.requestFrom(AM2315_I2CADDR, 8);
  for (uint8_t i=0; i<8; i++) {
    reply[i] = Wire1.read();
    //Serial.println(reply[i], HEX);
  }
  
  if (reply[0] != AM2315_READREG) return false;
  if (reply[1] != 4) return false; // bytes req'd
  
  coldSideHumidity = reply[2];
  coldSideHumidity *= 256;
  coldSideHumidity += reply[3];
  coldSideHumidity /= 10;
  //Serial.print("H"); Serial.println(humidity);

  coldSideTemp = reply[4] & 0x7F;
  coldSideTemp *= 256;
  coldSideTemp += reply[5];
  coldSideTemp /= 10;
  coldSideTemp = (coldSideTemp * 1.8) + 32;
  //Serial.print("T"); Serial.println(temp);

  // change sign
  if (reply[4] >> 7) coldSideTemp = -coldSideTemp;

  return true;
}




                                //Hot-Side Lamp Control Loop
void hotSideLamp_loop() {
  if(hotSideTemp >= 83 && hotSideTemp <= 87) {
    Serial.println("Hot-Side Temp is OK.");
    hotSideLampState = LOW;
    digitalWrite(hotSideLampPin, hotSideLampState);
  }

  if(hotSideTemp < 83) {
    Serial.println("Hot-Side Temp too Low. Turn on lamp");
    hotSideLampState = HIGH;
    digitalWrite(hotSideLampPin, hotSideLampState);
  }

  if(hotSideTemp > 87) {
    Serial.println("Hot-Side Temp too High. Turn off Lamp.");
    hotSideLampState = LOW;
    digitalWrite(hotSideLampPin, hotSideLampState);
  }
}



                                //Cold-Side Lamp Control Loop
void coldSideLamp_loop() {
  if(coldSideTemp >= 72 && coldSideTemp <= 78) {
    Serial.println("Cold-Side Temp is OK.");
    coldSideLampState = LOW;
    digitalWrite(coldSideLampPin, coldSideLampState);
  }

  if(coldSideTemp < 72) {
    Serial.println("Cold-Side Temp too Low. Turn on lamp.");
    coldSideLampState = HIGH;
    digitalWrite(coldSideLampPin, coldSideLampState);
  }


  if(coldSideTemp > 78) {
    Serial.println("Cold-Side Temp too High. Turn off Lamp.");
    coldSideLampState = LOW;
    digitalWrite(coldSideLampPin, coldSideLampState);
  }
}



                                //Mister Control Loop
void mister_loop() {
  if(hotSideHumidity >= 50 && hotSideHumidity <= 80 && coldSideHumidity >= 50 && coldSideHumidity <= 80) {
    Serial.println("Tank Humidity is OK.");
    misterState = LOW;
    digitalWrite(misterPin, misterState);
  }
  
  if(hotSideHumidity < 50 || coldSideHumidity < 50) {
    if(hotSideHumidity < 50){
      Serial.println("Hot-Side too dry. Turn on mister.");
    }
    if(coldSideHumidity < 50){
      Serial.println("Cold-Side too dry. Turn on mister.");
    }
    misterState = HIGH;
    digitalWrite(misterPin, misterState);
  }

  if(hotSideHumidity > 80 || coldSideHumidity > 80) {
    if(hotSideHumidity > 80) {
      Serial.println("Hot-Side too wet. Turn off mister.");
    }
    if(coldSideHumidity > 80) {
      Serial.println("Cold-Side too wet. Turn off mister.");
    }
    misterState = LOW;
    digitalWrite(misterPin, misterState);
  }
}



                             
void loop() {
  currentMillis = millis();
 
  if(currentMillis - previousMillis > 2000) {
    previousMillis = currentMillis;
    hotSideLamp_loop();
    coldSideLamp_loop();
    mister_loop();

//if(dataLog) {
      //Displaying Data from Sensor #1
      hotSideSensor();
      Serial.print("Hum #1: "); 
      Serial.print(hotSideHumidity);
      Serial.print("%");
      Serial.print("     Temp #1: "); 
      Serial.print(hotSideTemp);
      Serial.println(" Deg. F");
/*
      dataLog.print("Hum #1: "); 
      dataLog.print(hotSideHumidity);
      dataLog.print("%");
      dataLog.print("     Temp #1: "); 
      dataLog.print(hotSideTemp);
      dataLog.println(" Deg. F");
*/      
  
      //Displaying Data from Sensor #2  
      coldSideSensor();
      Serial.print("Hum #2: "); 
      Serial.print(coldSideHumidity);
      Serial.print("%");
      Serial.print("     Temp #2: "); 
      Serial.print(coldSideTemp);
      Serial.println(" Deg. F");
      Serial.println("");
/*
      dataLog.print("Hum #2: "); 
      dataLog.print(coldSideHumidity);
      dataLog.print("%");
      dataLog.print("     Temp #2: "); 
      dataLog.print(coldSideTemp);
      dataLog.println(" Deg. F");
      dataLog.println("");
    }*/
  }

}
