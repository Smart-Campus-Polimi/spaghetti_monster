#include <SPI.h>
#include <Wire.h>
#include <WiFi101.h>
#include <MQTT.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define SSID_WIFI "wlan_saltuaria"
#define PASS_WIFI "antlabpolitecnicomilano"
#define MQTT_BROKER "10.79.1.176"
#define MQTT_TOPIC "/hello"


/***************** SENSORS PINS ****************/
#define LIGHT_SENSOR A0//Grove - Light Sensor is connected to A0 
#define LIGHT_SENSOR_2 A1//Grove - Light Sensor is connected to A1 
#define DUST_SENSOR 8 //dust sensor
#define MQ7_SENSOR A2  //CO flying fish sensor (MQ7)
/***************** BME280 PINS ****************/
#define BME_SCK 9
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI


//float Rsensor; //Resistance of sensor in K for light sensor
int light_1;
int light_2;
int dustValue;
int MQ7Value;   // value read from the CO sensor 
unsigned long lowpulseoccupancy = 0; //for dust
float temperature;
float pressure;
float altitude;
float humidity;



typedef struct t  {
    unsigned long tStart;
    unsigned long tTimeout;
};

//Tasks and their Schedules.
t t_heat = {0, 150050}; //Run at beginning (60 sec)
t t_cool = {60000, 150050}; //Other 90 seconds
t t_read = {150000, 150050}; //final heat





WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  checkBME();
  client.begin(MQTT_BROKER, net);
  client.onMessage(messageReceived);
  pinMode(MQ7_SENSOR, INPUT);


  ensure_connections();
  starttime = millis();//get the current time;

}

void loop() {
  client.loop();

  if (!client.connected()) {
    ensure_connections();
  }

    duration = pulseIn(DUST_SENSOR, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;

    if (tCheck(&t_heat)) {
      client.publish("/hello", "world");
      heatReadMQ();
      readValues();
      printValues();
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

void printValues(){
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

      client.subscribe(MQTT_TOPIC);
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

