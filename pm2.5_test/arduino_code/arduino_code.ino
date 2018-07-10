// reference: https://www.dfrobot.com/wiki/index.php/PM2.5_laser_dust_sensor_SKU:SEN0177

//ATTENTION!!!   remember to set the baudrate of the Serial Monitor at 19200 bit/s

#include <Arduino.h>
#include <SPI.h>

#define LENG 31
//variables sensor
char buf[LENG];

//variables network & ThingSpeak
char ssid[] = "ssid-name"; // 
char pass[] = "password"; // 
unsigned long myChannelNumber = 450278;                                   //ThingSpeak channel ID
const char * myWriteAPIKey = "7COI7E0OLCD5O2KK";                          //ThingSpeak write API key


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


void setup(){
 //Serial.begin(9600);         //Serial used for print results on console

 //set serial sensor
 Serial1.begin(9600);         //Serial1 used for retrieve data coming frome the PM2.5 Sensor Adapter
 Serial1.setTimeout(1500); 
 Serial.println("try to connect");
}

void loop(){
  //Serial1.readBytes(buf, LENG);
  //Serial.println(buf);
    if(Serial1.find(0x42)){                   //start to read when detect 0x42
      Serial1.readBytes(buf,LENG);            //save the packet in a buffer

      if(buf[0] == 0x4d){
       if(checkValue(buf,LENG)){              //checking correctness of the packet
        int PM10Val = transmitPM10(buf);      
        int PM01Val = transmitPM01(buf);
        int PM25Val = transmitPM2_5(buf);
        
        Serial.print("PM10  = ");
        Serial.println(PM10Val);
        Serial.print("PM2.5 = ");
        Serial.println(PM25Val);
        Serial.print("PM1.0 = ");
        Serial.println(PM01Val);

        
      }else{
        Serial.println("error integrity packet");
      }
    } 
  }
  Serial.println("ok");
}

