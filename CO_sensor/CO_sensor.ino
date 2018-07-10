// MQ-7 wiring 
#define analogMQ7 A2      // Signal  
#define ledPin LED_BUILTIN      // Device internal LED       

int MQ7sensorValue = 0;   // value read from the sensor 

void setup() {
    Serial.begin(9600);
    pinMode(analogMQ7, INPUT);
    pinMode(ledPin, OUTPUT);
    delay(200);
    Serial.println("MQ-7 Gas Sensor Flying-Fish started");

   
}

// the loop routine runs over and over again forever:
void loop() {
    delay(200);
   
    COpreparation();
    MQ7sensorValue = COreading(); 
    printCOvalue(MQ7sensorValue);
}
int COreading(){
  // B) reading    
    // CO2 via MQ7: we need to read the sensor at 5V, but must not let it heat up. So hurry!
    Serial.println("reading..");
    analogWrite(analogMQ7, HIGH); 
    delay(50); // Getting an analog read apparently takes 100uSec
    return analogRead(analogMQ7);
}

void COpreparation(){
  // A) preparation
    // turn the heater fully on
   digitalWrite(ledPin, HIGH);

    Serial.println("heating");
    analogWrite(analogMQ7, HIGH); // HIGH = 255
    // heat for 1 min
    delay(60000);
    digitalWrite(ledPin, LOW);

    // now reducing the heating power: turn the heater to approx 1,4V
    Serial.println("reduce heating power");
    analogWrite(analogMQ7, 71.4); // 255x1400/5000
    // heat for 90 sec
    delay(90000);
  
}
void printCOvalue(int sensorValue){
   // C) print the results to the serial monitor
    Serial.print("MQ-7 PPM: ");                       
    Serial.println(sensorValue);  
    
  // D) interpretation
    // Detecting range: 20ppm-2000ppm carbon monoxide
    // air quality-cases: < 200 perfect, 200 - 800 normal, > 800 - 1800 high, > 1800 abnormal
    if (sensorValue <= 200) 
    {
        Serial.println("Air-Quality: CO perfect");
    }
    else if ((sensorValue > 200) || (sensorValue <= 800)) // || = or
    {
        Serial.println("Air-Quality: CO normal");
    }
    else if ((sensorValue > 800) || (sensorValue <= 1800))
    {
        Serial.println("Air-Quality: CO high");
    }
    else if (sensorValue > 1800) 
    {
        digitalWrite(ledPin, HIGH); // optical information in case of emergency
        Serial.println("Air-Quality: ALARM CO very high");
        delay(3000);
        digitalWrite(ledPin, LOW);
    }
    else
    {
        Serial.println("MQ-7 - cant read any value - check the sensor!");
    }
    Serial.println("\n");
}
