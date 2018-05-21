/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

  Dx nomenclature is defined within esp8266 core logic.  It can be used here as required
  NODE MCU Pinout
  D0 = 16 (Relay Output)
  D1 = 5 (OLED - SCL)
  D2 = 4 (OLED - SDA)
  D3 = 0 (DO NOT USE)    -  Can't use this pin.  It is used to reset the NodeMCU during program download
  D4 = 2 (DS18B20)
  D5 = 14 (PIR)
  D6 = 12 (Spare)
  D7 = 13 (DHT11)
  D8 = 15 (DO NOT USE) - must not be used with pullup resistor or logic will not download
  A0 = A0 (LDR)

  Remote Inputs
  P0 = (Door Closed)
  P1 = (Door Open)
  P2 = Spare
  P3 = Spare

  Remote Outputs
  P7 = Door Closed status (Green)
  P6 = Door Open status (Red)
  P5 = Motion Detected (Yellow)
  P4 = Spare (White)

  dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
*/
// ============================== WIFI CONFIGURATION ===========================
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
// Update these with values suitable for your network.
const char* ssid = "RedBear";
const char* password = "VLgregRy4h";
//const char* password = "dummy";
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Try to replace with DS18B20 MAC
// ============================== END WIFI CONFIGURATION =========================


// ============================== OTA CONFIGURATION ===========================
#include <ArduinoOTA.h>
int programMode = 0;
// ============================== END OTA CONFIGURATION =======================


// ============================= OLED DISPLAY CONFIGURATION ======================
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(OLED_RESET);
// This value must be specified in the Adafruit_SSD1306-master.h file
//#define SSD1306_128_32
// ============================== END OLED CONFIGURATION ======================


// ============================= SCREEN CONFIGURATION ======================
int screen = 0;    
int screenMax = 7;
bool screenChanged = true;   // initially we have a new screen,  by definition 
// defines of the screens to show
#define TIME                  0
#define GARAGETEMPERATURE     1
#define OUTSIDEHUMIDITY       2
#define OUTSIDETEMPERATURE    3
#define DOORSTATUS            4
#define LIGHTSTATUS           5
#define WIFISTATUS            6
#define MQTTSTATUS            7
long previousLCDMillis = 0;    // for LCD screen update
long lcdInterval = 2000;
// ============================== END SCREEN CONFIGURATION ===========================


// ============================== DHT11 CONFIGURATION ============================== 
#include <DHT.h>
#define DHTTYPE  DHT21
int dhtPin=D7;
DHT dht(dhtPin, DHTTYPE);
float t = 0;
float h = 0;
int failedCount = 0;
int failedCountLimit = 10;
// ============================== END DHT11 CONFIGURATION ===========================


// ============================== DS18B20 CONFIGURATION ======================
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into pin D4 on the Arduino
#define ONE_WIRE_BUS D4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
double DS18B20_temp = 0;
// ============================== END DS18B20 CONFIGURATION ======================


// ============================== MQTT CONFIGURATION =======================
WiFiClient espClient;
PubSubClient client(espClient);
const char* mqtt_server = "192.168.0.200";
// ============================== END MQTT CONFIGURATION ===========================


// ============================== GARAGE CONFIGURATION ======================
int relayPin = D0;

long doorOpenTime = 0;
int garageDoorOpenTimeLimit = 15*60*1000;

int ldrPin = A0;
bool lightStatus = false;
const int lightOnLimit = 400;
int ldrSensorValue;

int pirPin = D5;
bool pirMotionDetected = false;
long lastPIRReadTime = 0;
int sensorPIRReadInterval = 2*1000;      // Read PIR sensor every 5 seconds

ulong lastReadTime = 0;
int sensorReadInterval = 1000;
// ============================== END GARAGE CONFIGURATION ===========================


// ============================== TIME CONFIGURATION ======================
#include <TimeLib.h>

time_t prevDisplay = 0; // when the digital clock was displayed

// ============================== END TIME CONFIGURATION ===========================


// ============================== PCF8574 CONFIGURATION ======================
#include <PCF8574.h>

// Set i2c address
int pcf8574Address = 0x20;
PCF8574 pcf8574(pcf8574Address);
bool pcf8574Configured = false;

// ============================== END PCF8574 CONFIGURATION ===========================

// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void setupOLED() {
    // by default, we'll generate the high voltage from the 3.3v line internally!
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  
  Log("OLED Display setup complete!", "OLED\nSETUP");
}

// ============================ SETUP TEMP SENSOR =====================
//
// Description:
//
// ====================================================================
void setupDS18B20() {
  sensors.begin();
  getDeviceAddress();

  Log("DS18B20 setup complete!", "DS18B20\nSETUP");
}

// ============================ GET DEVICE ADDRESS =====================
//
// Description:
//
// ====================================================================
void getDeviceAddress(void) {
  byte dsAddress[8];
  
  Log( "Searching for DS18B20...", NULL);
  oneWire.reset_search();  // Start the search with the first device
  if( !oneWire.search(dsAddress) )
  {
    Log( "none found. Using default MAC address.",NULL);
  } else {
    Log( "success. Setting MAC address:", NULL);
    
    // Offset array to skip DS18B20 family code, and skip mac[0]
    mac[1] = dsAddress[3];
    mac[2] = dsAddress[4];
    mac[3] = dsAddress[5];
    mac[4] = dsAddress[6];
    mac[5] = dsAddress[7];
  }
  
  oneWire.reset_search();
  return;
}

// ============================ SETUP DHT SENSOR =====================
//
// Description:
//
// ====================================================================
void setupDHT() {
  dht.begin();
  Log("DHT setup complete!", "DHT\nSETUP");
}

// ============================ SETUP PIR SENSOR =====================
//
// Description:
//
// ====================================================================
void setupPIRSensor() {
  pinMode(pirPin, INPUT);
  Log("PIR setup complete!", "PIR\nSETUP");
}

// ============================ SETUP WIFI ============================
//
// Description:
//
// ====================================================================
void setupWifi() {
  bool connected = false;
  
  // We start by connecting to a WiFi network
  Log (strcat("Connecting to ", ssid), NULL);
  WiFi.mode(WIFI_STA); // <<< Station
  WiFi.begin(ssid, password);
  char i_msg[2];
  // Try to connect 10 times with 500 msec inbetween each attempt
  for (int i =0; i < 10; i++)
  {
    if (WiFi.status() != WL_CONNECTED) {
      dtostrf(i, 2, 0, i_msg);    // dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
      Log (strcat("Could not connect.  Attempt # ", i_msg), NULL);
      delay(500);
    } else {
      connected = true;
      continue;
    }
  }

  if (connected)
  {
    char *ip_address = IPtoCharArray(WiFi.localIP());
    Log (strcat("WiFi connected! IP address: ", ip_address), NULL);
  }
  else
  {
    Log (strcat("Could not connect to ", ssid), NULL);
  }

}

// ========================= IPtoCharArray =======================
//
// Description: Used to convert IP Address to char array for printing
//
// =====================================================================
char* IPtoCharArray(IPAddress _address)
{
    char szRet[20];
    String str = "";
    str += String(_address[0]);
    str += ".";
    str += String(_address[1]);
    str += ".";
    str += String(_address[2]);
    str += ".";
    str += String(_address[3]);
    str.toCharArray(szRet, 20);
    return szRet;
}

// ========================= SETUP GARAGE SENSOR =======================
//
// Description:
//
// =====================================================================
void setupGarageSensors() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  
  Log("Garage Sensors setup complete!", "GARAGE\nSETUP");
}

// ============================ SETUP MQTT ============================
//
// Description:
//
// ====================================================================
void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Log("MQTT setup complete!", "MQTT\nSETUP");
}

// ============================ SETUP PCF8574 ============================
//
// Description:
//
// ====================================================================
void setupPCF8574() {
    // The i2c_scanner uses the return value of the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(pcf8574Address);
    byte error = Wire.endTransmission();
 
    if (error != 0)
    {
      Log("Could not configure PCF8574!", "PCF8574\nERROR");
      pcf8574Configured = false;
    }

  // Set pinMode to OUTPUT
  pcf8574.pinMode(P0, INPUT);
  pcf8574.pinMode(P1, INPUT);
  pcf8574.pinMode(P2, INPUT);
  pcf8574.pinMode(P3, INPUT);
  pcf8574.pinMode(P4, OUTPUT);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT);
  pcf8574.pinMode(P7, OUTPUT);
  pcf8574.begin();

  pcf8574Configured = true;

  // Test LED outputs
  pcf8574.digitalWrite(P7, HIGH);
  delay(250);
  pcf8574.digitalWrite(P7, LOW);
  delay(250);
  pcf8574.digitalWrite(P6, HIGH);
  delay(250);
  pcf8574.digitalWrite(P6, LOW);
  delay(250);
  pcf8574.digitalWrite(P5, HIGH);
  delay(250);
  pcf8574.digitalWrite(P5, LOW);
  delay(250);
  pcf8574.digitalWrite(P4, HIGH);
  delay(250);
  pcf8574.digitalWrite(P4, LOW);
  delay(250);
  
  Log("PCF8574 setup complete!", "PCF8574\nSETUP");
}


// ============================ SETUP OTA ============================
//
// Description:
//
// ====================================================================
void setupOTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Log("OTA Start", NULL);
  });

  ArduinoOTA.onEnd([]() {
    Log("OTA End", NULL);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Log(strcat("Error: ", (char*)(long)error), NULL);
    if (error == OTA_AUTH_ERROR) Log("Auth Failed", NULL);
    else if (error == OTA_BEGIN_ERROR) Log("Begin Failed", NULL);
    else if (error == OTA_CONNECT_ERROR) Log("Connect Failed", NULL);
    else if (error == OTA_RECEIVE_ERROR) Log("Receive Failed", NULL);
    else if (error == OTA_END_ERROR) Log("End Failed", NULL);
  });

  ArduinoOTA.begin();
  Log("OTA setup complete!", "OTA/nSETUP");
}

// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-


// ============================ MQTT CALLBACK==========================
//
// Description:
//
// ====================================================================
void callback(char* topic, byte* payload, unsigned int length) {

  if (strcmp(topic,"garage/openDoor")==0){
      // Switch on the relay if a 1 was received as first character
      if ((char)payload[0] == '1') {
        closeGarageDoor();
      } 
  }

  if (strcmp(topic,"time")==0){
     payload[length] = '\0'; // Make payload a string by NULL terminating it.
     long Val = atol((char *)payload);
     setTime(Val);

     // Adjust to EST (and account for Daylight Saving Time)
     long adjustValue = 0;
     if (isDST(Val)) {
        adjustValue = 4 * SECS_PER_HOUR;
     } else {
        adjustValue = 5 * SECS_PER_HOUR;
     }
     adjustTime(-adjustValue);  // negative so that it subtracts time to get to EST
     Log("Time set", "TIME\nSET");
     
    }
  
  if (strcmp(topic,"garage/uploadCode")==0) {
      if ((char)payload[0] == '1') {
        Log ("Upload mode set", "UPLOAD");
        programMode = 1;
      }  
      
  }
}



// ============================ RECONNECT MQTT ========================
//
// Description:
//
// ====================================================================
void reconnect(char* msg_mqtt) {
  // Loop until we're reconnected
    // Try to connect 10 times with 500 msec inbetween each attempt
  for (int i =0; i < 10; i++)
  {
    if (!client.connected()) {
       Log("Attempting MQTT connection...", NULL);
      // Attempt to connect
      if (client.connect("ESP8266Client")) {
        Log("MQTT Connected", NULL);
        // Once connected, publish an announcement...
        client.publish("outTopic", "hello world");
        // ... and resubscribe
        client.subscribe("garage/openDoor");
        client.subscribe("time");
        client.subscribe("garage/uploadCode");
        strcpy(msg_mqtt, "CONNECTED");
      } else {
        Log("reconnect failed", NULL);
        strcpy(msg_mqtt, "MQTT FAIL");
        showErrorStatus(msg_mqtt);
        delay(250);
      }
    } 
  }

}


// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

// ============================ READ TIME ======================
//
// Description:
//
// ====================================================================
void readTime(char *msg_date, char *msg_time) {
  // TODO: Need to create temporary variables and re-assign them at the end of the function
  char temp[4] = {0};
  char t_msg_date[50] = {0};
  char t_msg_time[50] = {0};
  
  // year
  dtostrf(year(), 4, 0, t_msg_date);
  strcat(t_msg_date, "-");
  // month
  if(month() < 10){
    strcat(t_msg_date, "0");
    dtostrf(month(), 1, 0, temp);
  } else {
    dtostrf(month(), 2, 0, temp);
  }
  strcat(t_msg_date, temp);
  strcat(t_msg_date, "-");
  // day
  if(day() < 10){
    strcat(t_msg_date, "0");
    dtostrf(day(), 1, 0, temp);
  } else {
    dtostrf(day(), 2, 0, temp);
  }
  strcat(t_msg_date, temp);
  
  // hour
  /*if(hour() < 10){
    dtostrf(hour(), 1, 0, t_msg_time);
    //strcat(t_msg_time, "0");
  } else {*/
    dtostrf(hour(), 2, 0, temp);
  //}
  strcat(t_msg_time, temp);
  strcat(t_msg_time, ":");
  // minute
  if(minute() < 10){
    strcat(t_msg_time, "0");
    dtostrf(minute(), 1, 0, temp);
  } else {
    dtostrf(minute(), 2, 0, temp);
  }
  strcat(t_msg_time, temp);
  strcat(t_msg_time, ":");
  // second
  if(second() < 10){
    strcat(t_msg_time, "0");
    dtostrf(second(), 1, 0, temp);
  } else {
    dtostrf(second(), 2, 0, temp);
  }
  strcat(t_msg_time, temp);
  
  strcpy(msg_date, t_msg_date);
  strcpy(msg_time, t_msg_time);
}


// ============================ READ DOOR SENSORS ====================
//
// Description:
//
// ====================================================================
void readDoorSensor(char *msg_door) {

  /*char* openState = "OPEN";
  char* closedState = "CLOSED";
  char* inTransitState = "IN TRANSIT";
  */
  PCF8574::DigitalInput di;
  //readPCF8574Sensor(di);
  di = pcf8574.digitalReadAll();
  int result = 0;

  bool closedStatus = di.p0;
  bool openStatus = di.p1;
  result = (int)closedStatus + ((int)openStatus * -1);

  switch (result) {
      case -1:  // OPEN
          openStatus = true;
          strcpy(msg_door, "OPEN");
          //msg_door = openState;
          pcf8574.digitalWrite(P6,HIGH);  // Set OPEN HIGH
        break;
      case 0:   // IN TRANSIT
          openStatus = false;
          closedStatus = false;
          pcf8574.digitalWrite(P6,LOW);   // Set OPEN LOW
          pcf8574.digitalWrite(P7,LOW);   // Set CLOSED LOW
          strcpy(msg_door, "IN TRANSIT");
          //msg_door = inTransitState;
        break;
      case 1:   // CLOSED
          closedStatus = true;
          strcpy(msg_door, "CLOSED");
          //msg_door = closedState;
          pcf8574.digitalWrite(P7,HIGH);   // Set CLOSED HIGH
        break;
  }
  //Log (strcat("Door Status is: ", msg_door), NULL);
  
  client.publish("garage/doorStatus", msg_door);

}

// ============================ READ LDR SENSOR ======================
//
// Description:
//
// ====================================================================
void readLDRSensor(int &ldrSensorValue) {

    ldrSensorValue = analogRead(ldrPin);
    if (ldrSensorValue > lightOnLimit) {
      lightStatus = true;
      client.publish("garage/lightStatus", "ON");
    } else {
      lightStatus = false;
      client.publish("garage/lightStatus", "OFF");
    }

    char msg_light[50];
    dtostrf(ldrSensorValue,4,1,msg_light);
    client.publish("garage/lightlevel", msg_light);
}

// ============================ READ PIR SENSOR ======================
//
// Description:
//
// ====================================================================
void readPIRSensor() {
   if ((millis() - lastPIRReadTime) > sensorPIRReadInterval) {
      lastPIRReadTime = millis();
      pirMotionDetected = digitalRead(pirPin);

      if (pirMotionDetected == HIGH) {
        client.publish("garage/motionActive", "ON");
        pcf8574.digitalWrite(P5,HIGH);
      } else {
        client.publish("garage/motionActive", "OFF");
        pcf8574.digitalWrite(P5,LOW);
      }
   }
}

// ============================ READ DS18B20 TEMP SENSOR ======================
//
// Description:
//
// ====================================================================
void readDS18B20Sensor(char *msg_gt) {
    // read DS18B20 temperature sensor at index 0 (first sensor)
    sensors.requestTemperatures();  
    DS18B20_temp = sensors.getTempCByIndex(0);

    dtostrf(DS18B20_temp,4,1,msg_gt);
    client.publish("garage/garagetemp", msg_gt);
}

// ============================ READ DHT SENSOR =======================
//
// Description:
//
// ====================================================================
void readDHT11Sensor(char *msg_ot, char *msg_oh) {
    float prev_t = t;
    float prev_h = h;
    if (failedCount < failedCountLimit){
      t = dht.readTemperature();
      h = dht.readHumidity();
      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t) ) {
        Log("Failed to read from DHT sensor!",NULL);
        t = prev_t;
        h = prev_h;
        failedCount++;
      } else {
        failedCount = 0;
      }
    }
    
    dtostrf(t,4,1,msg_ot);
    client.publish("garage/outsidetemp", msg_ot);
    
    dtostrf(h,4,1,msg_oh);
    client.publish("garage/outsidehumidity", msg_oh);

}

// ============================ READ PCF8574 SENSOR =======================
//
// Description:
//
// ====================================================================
/*void readPCF8574Sensor(PCF8574::DigitalInput &di) {
  PCF8574::DigitalInput returnValue = pcf8574.digitalReadAll();
  di = returnValue;
}*/


// ============================ PUBLISH TIME DATA =======================
//
// Description:
//
// ====================================================================
void publishTime(char msg_date[], char msg_time[]) {
  char date_time [50] = "2018";
  strcpy(date_time, msg_date);
  strcat(date_time, "-");
  strcat(date_time, msg_time);
  
  client.publish("garage/time", date_time);
}


// ====================== Display Method =================
// 
// Method that is called to display the data on the OLED
// 
// ===================================================================
void displayScreen(const char *msg_gt, const char* msg_oh, const char* msg_ot, const char* msg_door, const int ldrSensorValue, const char* msg_wifi, const char* msg_mqtt) {
  unsigned long currentLCDMillis = millis();
  
  // MUST WE SWITCH SCREEN?
  if(currentLCDMillis - previousLCDMillis > lcdInterval)              // save the last time you changed the display 
  {
    previousLCDMillis = currentLCDMillis; 
    screen++;
    if (screen > screenMax) screen = 0;  // all screens done? => start over
    screenChanged = true; 
  }

  // DISPLAY CURRENT SCREEN
  if (screenChanged)//   -- only update the screen if the screen is changed.
  {
    screenChanged = false; // reset for next iteration
    switch(screen)
    {
    case GARAGETEMPERATURE: 
      showGarageTemperature(msg_gt); 
      break;
    case OUTSIDEHUMIDITY: 
      showOutsideHumidity(msg_oh);
      break;
    case OUTSIDETEMPERATURE: 
      showOutsideTemperature(msg_ot); 
      break;
    case DOORSTATUS:
      showDoorStatus(msg_door);
      break;
    case LIGHTSTATUS:
      showLightStatus(ldrSensorValue);
      break;
    case TIME:
      showTime();
      break;
    case WIFISTATUS:
      showWifiStatus(msg_wifi);
      break;
    case MQTTSTATUS:
      showMQTTStatus(msg_mqtt);
      break;
    default:
      showErrorStatus(NULL);
      // cannot happen -> showError() ?
      break;
    }
  }
}


// ============================ CLEAR DISPLAY ===============================
//
// Description: Setup display.  Called at the begining of each display function
//
// ===========================================================================
void clearDisplay() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
}


// ============================ SHOW OUTSIDE TEMPERATURE DISPLAY ============
//
// Description:
//
// ===========================================================================
void showOutsideTemperature(const char *msg_ot)
{
  clearDisplay();
       
  display.println("OUTSIDE:");
  display.print(msg_ot); // DHT21 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}


// ============================ SHOW OUTSIDE HUMIDITY DISPLAY ================
//
// Description:
//
// ===========================================================================
void showOutsideHumidity(const char *msg_oh)
{
  clearDisplay();
       
  display.println("HUMIDITY:");
  display.print(msg_oh);
  display.println("%");

  display.display();
}


// ============================ SHOW GARAGE TEMPERATURE DISPLAY ==============
//
// Description:
//
// ===========================================================================
void showGarageTemperature(const char *msg_gt)
{
  clearDisplay();
       
  display.println("GARAGE:");
  display.print(msg_gt); // DS18B20 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}


// ============================ SHOW GARAGE DOOR STATUS DISPLAY ===============
//
// Description:
//
// ===========================================================================
void showDoorStatus(const char *msg_door)
{
  clearDisplay();
       
  display.println("DOOR");
  display.println(msg_door);

  display.display();
}


// ============================ SHOW LIGHT STATUS DISPLAY ===================
//
// Description:
//
// ===========================================================================
void showLightStatus(const int ldrSensorValue)
{
  clearDisplay();
       
  display.println("LIGHT");

  if (lightStatus == true)
    display.print("ON");
  else
    display.print("OFF");

  display.print(" (");
  display.print(ldrSensorValue);
  display.println(")");
  
  display.display();
}

// ============================ SHOW TIME DISPLAY ===========================
//
// Description:
//
// ===========================================================================
void showTime()
{
  clearDisplay();

  // digital clock display of the time
  display.print(year()); 
  display.print("-");
  display.print(month());
  display.print("-");
  display.println(day());

  display.print(hour());
  display.print(":");
  if(minute() < 10)
    display.print('0');
  display.print(minute());
  display.print(":");
  if(second() < 10)
    display.print('0');
  display.print(second());

  display.display();
}


// ============================ SHOW WIFI STATUS DISPLAY ===============
//
// Description:
//
// ===========================================================================
void showWifiStatus(const char *msg_wifi)
{
  clearDisplay();
       
  display.println("WIFI");
  display.println(msg_wifi);

  display.display();
}


// ============================ SHOW MQTT STATUS DISPLAY ===============
//
// Description:
//
// ===========================================================================
void showMQTTStatus(const char *msg_mqtt)
{
  clearDisplay();
       
  display.println("MQTT");
  display.println(msg_mqtt);

  display.display();
}


// ============================ SHOW ERROR STATUS DISPLAY ===============
//
// Description:
//
// ===========================================================================
void showErrorStatus(const char *msg_error)
{
  clearDisplay();
       
  display.println(msg_error);

  display.display();
}


// ============================ SHOW OPEN / CLOSE GARAGE DOOR DISPLAY ================
//
// Description:
//
// ===========================================================================
void showOpenCloseGarageDoor()
{
  clearDisplay();
       
  display.println("OPEN/CLOSE");
  display.println("DOOR");

  display.display();
}


// ============================ CHECK GARAGE DOOR STATUS ===============
//
// Description:
//
// ======================================================================
void CheckGarageDoorStatus(const char *msg_door) {

  if (msg_door != "OPEN") {    // Door is closed or in transit, nothing else to do
    doorOpenTime = millis();
    return;
  }

  if (pirMotionDetected) {
      doorOpenTime = millis();
      return;
  }
  
  long garageDoorTimeElapsed = millis() - doorOpenTime;

  
  if (garageDoorTimeElapsed > garageDoorOpenTimeLimit) {    // 15 minutes
    closeGarageDoor();
    doorOpenTime = millis();
  }
}

// ============================ OPEN/CLOSE GARAGE DOOR ==================
//
// Description:
//
// ======================================================================
void closeGarageDoor() {
  showOpenCloseGarageDoor();
  digitalWrite(relayPin, HIGH);   // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because it is acive low on the ESP-01)
  delay(1000);
  digitalWrite(relayPin, LOW);  // Turn the LED off by making the voltage HIGH
  Log("Send Open/Close Garage Door Command!", "CLOSE\nDOOR"); 
}

// ============================ LOG MESSAGES ==================
//
// Description:
//
// ======================================================================
void Log(const char *message, const char *displayMessage) {
  Serial.println(message); 
  
  //if (client.connected())
    client.publish("garage/message", message);
  
  // could use strncpy to copy 10 characters for each line to avoid overflow on OLED
  if (sizeof(displayMessage) >0) {    // or use strlen()
    clearDisplay();
    display.println(displayMessage);
    display.display();
    //previousLCDMillis = millis();
  }
}

// ============================ SETUP ==================================
//
// Description:
//
// ====================================================================
void setup() {
  Serial.begin(9600);
  
  // --------- setup OLED  --------- 
  setupOLED();

  // --------- setup DS18B20 Temp Sensor --------- 
  setupDS18B20();
  
  // --------- setup Output Relay  --------- 
  setupGarageSensors();

  // --------- setup Wifi Connection  --------- 
  setupWifi();

  // --------- setup MQTT  --------- 
  setupMQTT();
  
  // --------- setup DHT  --------- 
  setupDHT();

  // --------- setup PIR  --------- 
  setupPIRSensor();
  
  // --------- setup PCF8574  --------- 
  setupPCF8574();

  // --------- setup OTA  --------- 
  setupOTA();


  Log("SETUP COMPLETED SUCCESSFULLY!", "SETUP\nCOMPLETE");
}

// ============================ LOOP ==================================
//
// Description:
//
// ====================================================================
void loop() {
bool wifiConnected = false;
char msg_gt[50];
char msg_oh[50];
char msg_ot[50];
char msg_door[50];
//char msg_motion[50];
int ldrSensorValue;
char msg_date[50];
char msg_time[50];
char msg_wifi[50];
char msg_mqtt[50];

  if (WiFi.status() != WL_CONNECTED) {
    strcpy(msg_wifi, "DISCONNECTED");
    strcpy(msg_mqtt, "DISCONNECTED");
    wifiConnected = false;
  } else {
    strcpy(msg_wifi, "CONNECTED");
    wifiConnected = true;
  }

  if (!client.connected() && wifiConnected) {
    reconnect(msg_mqtt);
  }
  client.loop();

  if (programMode == 1) {
    ArduinoOTA.handle();
  } else {
      if (timeStatus() != timeNotSet) {
          if (now() != prevDisplay) { //update the display only if time has changed
            prevDisplay = now();
          }
        }
        
        if (millis() - lastReadTime > sensorReadInterval) {
          lastReadTime = now();
          readTime(msg_date, msg_time);
          
          if (pcf8574Configured == true) {
            readDoorSensor(msg_door);
          }

          readLDRSensor(ldrSensorValue);

          readPIRSensor();

          readDS18B20Sensor(msg_gt);

          readDHT11Sensor(msg_ot, msg_oh);

          //CheckGarageDoorStatus(msg_door);

          publishTime(msg_date, msg_time);

        }
        
        displayScreen(msg_gt, msg_oh, msg_ot, msg_door, ldrSensorValue, msg_wifi, msg_mqtt);
    }
 
}
