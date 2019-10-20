/* Example sketch for AM2315 humidity - temperature sensor
   Written by cactus.io, and requires the cactus_io_AM2315 library. public domain
   This sketch will work with the Adafruit AM2315 sensor.
   For hookup details using this sensor then visit
   http://cactus.io/hookups/sensors/temperature-humidity/am2315/hookup-arduino-to-am2315-temp-humidity-sensor
*/

#include <Wire.h>
#include "cactus_io_AM2315.h"

AM2315 am2315;

void setup() {
  Serial.begin(9600);
  Serial.println("AM2315 Humidity - Temperature Sensor");
  Serial.println("RH\t\tTemp (C)\tTemp (F)");
 

  if (!am2315.begin()) {
     Serial.println("Sensor not found, check wiring & pullups!");
     while (1);
  }
}

void loop() {
  
  am2315.readSensor();
  
  Serial.print(am2315.getHumidity()); Serial.print(" %\t\t");
  Serial.print(am2315.getTemperature_C()); Serial.print(" *C\t");
  Serial.print(am2315.getTemperature_F()); Serial.println(" *F\t");

  // Add a 2 second
  delay(2000);
}
