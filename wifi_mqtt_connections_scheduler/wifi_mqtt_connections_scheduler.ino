#include <SPI.h>
#include <Wire.h>
#include <WiFi101.h>
//#include <MQTT.h>
#include <PubSubClient.h>

#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <ArduinoJson.h>




#define SSID_WIFI "wlan_saltuaria"
#define PASS_WIFI "antlabpolitecnicomilano"
#define MQTT_BROKER "10.79.1.176"
#define MQTT_TOPIC "/envi/1"


/***************** SENSORS PINS ****************/
#define LIGHT_SENSOR A0//Grove - Light Sensor is connected to A0 
#define LIGHT_SENSOR_2 A1//Grove - Light Sensor is connected to A1 
#define DUST_SENSOR 8 //dust sensor
#define MQ7_SENSOR A2  //CO flying fish sensor (MQ7)
#define SOUND_SENSOR A6
/***************** BME280 PINS ****************/
#define BME_SCK 9
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes for PM10


Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
char buf[LENG];
//char jsonChar[120];



//float Rsensor; //Resistance of sensor in K for light sensor
int light_1;
int light_2;
int dustValue;
int MQ7Value;   // value read from the CO sensor 
int soundValue;
//dust
unsigned long lowpulseoccupancy = 0; 
unsigned long duration;
unsigned long sampletime_ms = 150050;//sampe 1s ;

float temperature;
float pressure;
float altitude;
float humidity;
int PM01Value=0;          //define PM1.0 value of the air detector module
int PM2_5Value=0;         //define PM2.5 value of the air detector module
int PM10Value=0;         //define PM10 value of the air detector module
int sequenceNumber = 0;


typedef struct t  {
    unsigned long tStart;
    unsigned long tTimeout;
};

//Tasks and their Schedules.
t t_heat = {0, 15005}; //Run at beginning (60 sec)
t t_cool = {6000, 15005}; //Other 90 seconds
t t_read = {15000, 15005}; //final heat


WiFiClient net;
PubSubClient client(net);
StaticJsonBuffer<500> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();

String sPayload;
char* cPayload;


void setup() {
  Serial.begin(115200);
  delay(100);
  checkBME();
  Serial1.begin(9600);         //Serial1 used for retrieve data coming frome the PM2.5 Sensor Adapter
  Serial1.setTimeout(1500); 
  pinMode(MQ7_SENSOR, INPUT);

  client.setServer(MQTT_BROKER, 1883);
  //client.begin(MQTT_BROKER, net);

  ensure_connections();

}

void loop() {
  client.loop();

  if (!client.connected()) {
    ensure_connections();
  }

    duration = pulseIn(DUST_SENSOR, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;

    if (tCheck(&t_heat)) {
      heatReadMQ();
      readValues();
      createJson();
      //printValues();
      //convert in json
      char jsonChar[200];
      root.printTo(jsonChar, sizeof(jsonChar)); 
      
      if (client.publish(MQTT_TOPIC, jsonChar) == true) {
         Serial.println("Success sending message");
      } else {
         Serial.println("Error sending message");
      }
      print_time(millis());
      tRun(&t_heat);
    }
    
    if (tCheck(&t_cool)) {
      cool();
      print_time(millis());
      tRun(&t_cool);
    }
    
    if (tCheck(&t_read)) {
      miniHeat();
      print_time(millis());
      tRun(&t_read);
    }
   
}

void heatReadMQ(){
   MQ7Value = analogRead(MQ7_SENSOR);
   Serial.print("Heating MQ7");
   analogWrite(MQ7_SENSOR, HIGH); // HIGH = 255
}

void cool(){
    Serial.print("Cooling MQ7");
    analogWrite(MQ7_SENSOR, 71.4); // 255x1400/5000

}

void miniHeat(){
    Serial.print("New mini-heat for MQ7");
    analogWrite(MQ7_SENSOR, HIGH);
}

void readValues(){
  //light
  light_1 = analogRead(LIGHT_SENSOR); 
  light_2 = analogRead(LIGHT_SENSOR_2); 
  //dust
  dustValue = readDust();
  //BME
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  humidity = bme.readHumidity();
  //sound
  soundValue = analogRead(SOUND_SENSOR);
  
    
}

int readDust(){
    float ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
    float concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
 
    if (concentration > 20000 && concentration < 315000) {
     Serial.println("Smokes from matches detected!"); 
    
    }
      if (concentration > 315000) {
     Serial.println("Smokes from cigarettes detected! Or It might be a huge fire! Beware!"); 
    
    }
    
    lowpulseoccupancy = 0;

    return concentration;
}

void createJson(){
  root["SN"] = sequenceNumber;
  root["light_1"] = light_1;
  root["light_2"] = light_2;
  root["dust"] = dustValue;
  root["CO"] = MQ7Value;
  root["temperature"] = temperature;
  root["presssure"] = pressure;
  root["altitude"] = altitude;
  root["humidity"] = humidity;
  root["sound"] = soundValue;
  root["pm01"] = PM01Value;
  root["pm2_5"] = PM2_5Value;
  root["pm10"] = PM10Value;

  sequenceNumber++;
  root.prettyPrintTo(Serial);
}

void printValues(){
    Serial.println("\n\n");
    //LIGHT
    Serial.print("The light values are: ");
    Serial.print(light_1);
    Serial.print(", ");
    Serial.println(light_2);

    //DUST
    Serial.print("The dust value is: ");
    Serial.print(dustValue);
    Serial.println(" pcs/0.01cf  ");

    //CO
    Serial.print("The CO value is: ");
    Serial.println(MQ7Value);
    //check for warnings (too much CO)

    //BME
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    //sound
    Serial.print("Sound:");
    Serial.println(soundValue);

    //PM
    Serial.print("PM1.0: ");  
    Serial.print(PM01Value);
    Serial.println("  ug/m3");            
    
    Serial.print("PM2.5: ");  
    Serial.print(PM2_5Value);
    Serial.println("  ug/m3");     
      
    Serial.print("PM1 0: ");  
    Serial.print(PM10Value);
    Serial.println("  ug/m3");   

}

void ensure_connections(){
  if (wifi_connect()){
    if(!mqtt_connect()){
      Serial.println("very big MQTT problem, restart");
      while(1); //block the program
    }
  }
  else {
    Serial.println("very big Wi-Fi problem, restart");
    while(1); //block the program
  }
}

boolean wifi_connect() {
  Serial.println("Connecting to ");
  Serial.print(SSID_WIFI);
  WiFi.begin(SSID_WIFI, PASS_WIFI);

  Serial.print("checking wifi..");
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    i++;
    if (i == 20) {
      Serial.println("WI-FI Connection Error!!!!");
      return 0;                     //too much time to connect, given up
    }
  }
  Serial.println("WiFi connected");
  return 1;  
}

boolean mqtt_connect() {
  int i = 0;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("my_mkr")) {
      Serial.println("connected");
      return 1;
    }
    else {
      Serial.print("failed mqtt");
      if (i > 1) {
        Serial.println("impossible to connect to MQTT");
        return 0;
      }
      Serial.println(" try again in 5 seconds");
      delay(3000);
    }
    i++;
  }
}

bool tCheck (struct t *t ) {
   if (millis() > t->tStart + t->tTimeout) {
    return true;  
  }
  else {
    return false;
    } 
}

void tRun (struct t *t) {
    t->tStart = millis();
}

void print_time(unsigned long time_millis){
    Serial.print("Time: ");
    Serial.print(time_millis/1000);
    Serial.println("s");
}

void checkBME(){
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

//function used for checking if the packet sent by the sensor is correct
char checkValue(char *thebuf, char leng)
{  
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;
 
  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

//retrieve pm10 value
int transmitPM10(char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); 
  return PM10Val;
}

//retrieve pm1.0 value
int transmitPM01(char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); 
  return PM01Val;
}

//retrieve pm2.5 value
int transmitPM2_5(char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);
  return PM2_5Val;
}

void readPM(){
   if(Serial1.find(0x42)){                   //start to read when detect 0x42
      Serial1.readBytes(buf,LENG);            //save the packet in a buffer

      if(buf[0] == 0x4d){
       if(checkValue(buf,LENG)){              //checking correctness of the packet
        PM01Value = transmitPM10(buf);      
        PM2_5Value = transmitPM01(buf);
        PM10Value = transmitPM2_5(buf);
       
      }else{
        Serial.println("error integrity packet");
      }
    } 
  }
}

