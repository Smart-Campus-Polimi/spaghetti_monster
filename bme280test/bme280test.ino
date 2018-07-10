/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK 9
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SHORT_DELAY 5000

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

float temperature;
float pressure;
float altitude;
float humidity;

void setup() {
    Serial.begin(9600);
    delay(500);
    Serial.println("BME280 test");

    delay(20);
    bool status;
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1){
          Serial.println("Restart the system!");
        }
    }
    Serial.println();
}


void loop() { 
    readValues();
    printValues();
    delay(SHORT_DELAY);
}

void readValues(){
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();
  
}

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.println();
}
