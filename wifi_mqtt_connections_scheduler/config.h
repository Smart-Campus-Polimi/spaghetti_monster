/*
WIFI 
  1 wlan_salt *
  2 iotpolimi
  3 vodafone 5g
*/
/*
BROKER
1 aws old
2 cluster
3 aws uc15 *
*/
#define WIFI 1 
#define BROKER 3 
#define MQTT_TOPIC "smart_campus/environmental/antlab"
#define AVG_MES 120

/*********** TIME CONFIG **************/
#define START_TIME 0
#define TOTAL_TIME 1 //seconds

/*********** END TIME  **************/



/*********** WIFI CONFIG ***********/
#if WIFI == 1
  #define SSID_WIFI "wlan_saltuaria"
  #define PASS_WIFI "antlabpolitecnicomilano"
#endif

#if WIFI == 2
  #define SSID_WIFI "IoTPolimi"
  #define PASS_WIFI "ZpvYs=gT-p3DK3wb"
#endif

#if WIFI == 3
  #define SSID_WIFI "HUAWEI-5GCPE-D858"
  #define PASS_WIFI "Vodafone5G"
#endif
/*********** END WIFI ***********/

/*********** BROKER CONFIG ***********/
#if BROKER == 1
  #define MQTT_BROKER "ec2-35-166-12-244.us-west-2.compute.amazonaws.com"
#endif

#if BROKER == 2
  #define MQTT_BROKER "10.79.1.176"
#endif

#if BROKER == 3
  #define MQTT_BROKER "34.222.27.8"
#endif
/*********** END BROKER ***********/
