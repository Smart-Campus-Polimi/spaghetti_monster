#include <SPI.h>
#include <WiFi101.h>
#include <MQTT.h>
#include <math.h>


#define SSID_WIFI "wlan_saltuaria"
#define PASS_WIFI "antlabpolitecnicomilano"
#define MQTT_BROKER "10.79.1.176"
#define MQTT_TOPIC "/hello"


/***************** SENSORS PINS ****************/
#define LIGHT_SENSOR A0//Grove - Light Sensor is connected to A0 
#define LIGHT_SENSOR_2 A1//Grove - Light Sensor is connected to A1 
#define DUST_SENSOR 8 //dust sensor
#define analogMQ7 A2  //CO flying fish sensor (MQ7)


//float Rsensor; //Resistance of sensor in K for light sensor
int light_1;
int light_2;
int dustValue;
int MQ7Value;   // value read from the CO sensor 


//for dust
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 10000;//sampe 1s ;
unsigned long lowpulseoccupancy = 0;


WiFiClient net;
MQTTClient client;

unsigned long lastMillis = 0;


void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  client.begin(MQTT_BROKER, net);
  client.onMessage(messageReceived);
  pinMode(analogMQ7, INPUT);


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
  
  // publish a message roughly every second.
  if ((millis()-starttime) > sampletime_ms){
    Serial.println(millis()-starttime);
    client.publish("/hello", "world");
    readValues();
    printValues();

    starttime = millis();
  }
  
}

void readValues(){
  //light
  light_1 = analogRead(LIGHT_SENSOR); 
  light_2 = analogRead(LIGHT_SENSOR_2); 
  //dust
  dustValue = readDust();
    
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
    Serial.print("the light values are: ");
    Serial.print(light_1);
    Serial.print(", ");
    Serial.println(light_2);

    Serial.print("the dust value is: ");
    Serial.print(dustValue);
    Serial.println(" pcs/0.01cf  ");
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

