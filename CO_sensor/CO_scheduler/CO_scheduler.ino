// MQ-7 wiring 
#define analogMQ7 A2      // Signal  
#define ledPin LED_BUILTIN      // Device internal LED       


#define INTERVAL_MESSAGE1 5000
#define INTERVAL_MESSAGE2 7000
#define INTERVAL_MESSAGE3 11000
#define INTERVAL_MESSAGE4 13000
 
unsigned long time_1 = 0;
unsigned long time_2 = 0;
unsigned long time_3 = 0;
unsigned long time_4 = 0;
 
void print_time(unsigned long time_millis);

int MQ7sensorValue = 0;   // value read from the sensor 

void setup() {
    Serial.begin(9600);
    pinMode(analogMQ7, INPUT);
    pinMode(ledPin, OUTPUT);
    delay(200);
    Serial.println("MQ-7 Gas Sensor Flying-Fish started");

}


void loop() {
  // put your main code here, to run repeatedly:

}

void print_time(unsigned long time_millis){
    Serial.print("Time: ");
    Serial.print(time_millis/1000);
    Serial.print("s - ");
}
