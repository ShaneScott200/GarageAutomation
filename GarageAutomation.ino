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
  D3 = 0 (PIR)     -  Can't use this pin.  It is used to reset the NodeMCU during program download
  D4 = 2 (DS18B20)
  D5 = 14 (Door Open)
  D6 = 12 (DHT11)
  D7 = 13 (Door Closed)
  D8 = 15 (NOT USED) - must not be used with pullup resistor or logic will not download
  A0 = A0 (LDR)
*/

// ============================== WIFI CONFIGURATION ===========================
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
// Update these with values suitable for your network.
const char* ssid = "RedBear";
const char* password = "VLgregRy4h";
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Try to replace with DS18B20 MAC
// ============================== END WIFI CONFIGURATION =========================


// ============================== OTA CONFIGURATION ===========================
#include <ArduinoOTA.h>
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
int screenMax = 5;
bool screenChanged = true;   // initially we have a new screen,  by definition 
// defines of the screens to show
#define GARAGETEMPERATURE     0
#define OUTSIDEHUMIDITY       1
#define OUTSIDETEMPERATURE    2
#define DOORSTATUS            3
#define LIGHTSTATUS           4
#define TIME                  5
long previousLCDMillis = 0;    // for LCD screen update
long lcdInterval = 2000;
// ============================== END SCREEN CONFIGURATION ===========================


// ============================== DHT11 CONFIGURATION ============================== 
#include <DHT.h>
#define DHTTYPE  DHT21
int dhtPin=D6;
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
#define ONE_WIRE_BUS D3
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
double DS18B20_temp = 0;
// ============================== END DS18B20 CONFIGURATION ======================

// ============================== MQTT CONFIGURATION =======================
//#include<stdlib.h>
WiFiClient espClient;
PubSubClient client(espClient);
const char* mqtt_server = "192.168.0.101";
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

long lastReadTime = 0;
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
  
  showInit();
  // init done
  Serial.println("OLED Display Initialized!");
}

// ============================ SETUP TEMP SENSOR =====================
//
// Description:
//
// ====================================================================
void setupDS18B20() {
  sensors.begin();
  getDeviceAddress();
}

// ============================ GET DEVICE ADDRESS =====================
//
// Description:
//
// ====================================================================
void getDeviceAddress(void) {
  byte i;
  byte dsAddress[8];
  
  Serial.println( "Searching for DS18B20..." );
  oneWire.reset_search();  // Start the search with the first device
  if( !oneWire.search(dsAddress) )
  {
    Serial.println( "none found. Using default MAC address." );
  } else {
    Serial.println( "success. Setting MAC address:" );
    Serial.print( " DS18B20 ROM  =" );
    for( i = 0; i < 8; i++)
    {
      Serial.write(' ');
      Serial.print( dsAddress[i], HEX );
    }
    Serial.println();
    
    // Offset array to skip DS18B20 family code, and skip mac[0]
    mac[1] = dsAddress[3];
    mac[2] = dsAddress[4];
    mac[3] = dsAddress[5];
    mac[4] = dsAddress[6];
    mac[5] = dsAddress[7];
  }
  
  Serial.print( " Ethernet MAC =" );
  for( i = 0; i < 6; i++ )
  {
    Serial.write( ' ' );
    Serial.print( mac[i], HEX );
  }
  Serial.println();
  
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
}

// ============================ SETUP PIR SENSOR =====================
//
// Description:
//
// ====================================================================
void setupPIRSensor() {
  pinMode(pirPin, INPUT);
}

// ============================ SETUP WIFI ============================
//
// Description:
//
// ====================================================================
void setupWifi() {
  bool connected = false;
  
  // We start by connecting to a WiFi network
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.mode(WIFI_STA); // <<< Station
  WiFi.begin(ssid, password);

  // Try to connect 10 times with 500 msec inbetween each attempt
  for (int i =0; i < 10; i++)
  {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("Could not connect.  Attempt #"); Serial.println(i);
      delay(500);
    } else {
      connected = true;
      continue;
    }
  }

  if (connected)
  {
    Serial.print("WiFi connected! "); Serial.print("IP address: "); Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println(" ");
    Serial.print("Could not connect to ");
    Serial.println(ssid);
  }

}

// ========================= SETUP GARAGE SENSOR =======================
//
// Description:
//
// =====================================================================
void setupGarageSensors() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Set high to disable relay
  //pinMode(openStatusPin, INPUT);
  //pinMode(closedStatusPin, INPUT);
}

// ============================ SETUP MQTT ============================
//
// Description:
//
// ====================================================================
void setupMQTT() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
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
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (pcf8574Address<16) {
        Serial.print("0"); 
      }
      Serial.print(pcf8574Address,HEX);
      Serial.println("  !");
      pcf8574Configured = true;
    } else {
      Serial.println("Could not configure PCF8574");
      pcf8574Configured = false;
    }

  if (pcf8574Configured == true) {
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
  }
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
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-


// ============================ MQTT CALLBACK==========================
//
// Description:
//
// ====================================================================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println();
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic,"garage/openDoor")==0){
      // Switch on the relay if a 1 was received as first character
      if ((char)payload[0] == '1') {
        closeGarageDoor();
      } else {
        digitalWrite(relayPin, HIGH);  // Turn the LED off by making the voltage HIGH
      }
  }

  if (strcmp(topic,"time")==0){
      // Switch on the relay if a 1 was received as first character
      // parse time string from 'payload'  format is: "yyyy-mm-dd-hr-min-sec"
      int timeValues[] = {2018,3,23,12,05,0};         // Initialize time array with a default time.  If there is an error the value will be 0.
      char instring[length];                          // create the array to hold the payload converted to char (required for strtok)
      for (int i = 0; i<length; i++) {                // convert the payload to char array
        instring[i] = (char)payload[i];
      }
      char delimiters[] = "-";                        // Could also add ‘:’ for time
      const  int MAXVALUES = 6;
      char* valPosition;
     
      //This initializes strtok with our string to tokenize
      valPosition = strtok(instring, delimiters);       // Get the first item from the payload
      for (int i  = 0; i < MAXVALUES; i++){             // Populate the array with the remaining values in the payload
        timeValues[i] = atoi(valPosition);              // convert value to int and store time values in array.
        //Here we pass in a NULL value, which tells strtok to continue working with the previous string
        valPosition = strtok(NULL, delimiters);
      }
      
     // set the time
     setTime(timeValues[3],timeValues[4],timeValues[5],timeValues[2],timeValues[1],timeValues[0]);
     Serial.println("Time set");
    }
}



// ============================ RECONNECT MQTT ========================
//
// Description:
//
// ====================================================================
void reconnect() {
  // Loop until we're reconnected
    // Try to connect 10 times with 500 msec inbetween each attempt
  for (int i =0; i < 10; i++)
  {
    if (!client.connected()) {
       Serial.print("Attempting MQTT connection...");
      // Attempt to connect
      if (client.connect("ESP8266Client")) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish("outTopic", "hello world");
        // ... and resubscribe
        client.subscribe("garage/openDoor");
        client.subscribe("time");
      } else {
        Serial.println("reconnect failed");
        delay(500);
      }
    } 
  }
  /*
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("garage/openDoor");
      client.subscribe("time");
    } else {
      Serial.println("failed, rc=");
      delay(5000);
    }
  }*/
}


// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
// *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-


// ============================ READ DOOR SENSORS ====================
//
// Description:
//
// ====================================================================
void readDoorSensor(char *msg_door) {
      
  PCF8574::DigitalInput di;
  readPCF8574Sensor(di);

  bool closedStatus = di.p0;
  bool openStatus = di.p1;
  
  // read door status
  if (openStatus == HIGH) {
    openStatus = true;
    //msg_door = {"OPEN"};
    strcpy(msg_door, "OPEN");
    pcf8574.digitalWrite(P5,HIGH);  // Set OPEN HIGH
  } else {
    if (closedStatus == HIGH) {
      closedStatus = true;
      //msg_door = "CLOSED";
      strcpy(msg_door, "CLOSED");
      pcf8574.digitalWrite(P4,HIGH);   // Set CLOSED HIGH
    } else {
      closedStatus = false;
      //msg_door = "IN TRANSIT";
      strcpy(msg_door, "IN TRANSIT");
      pcf8574.digitalWrite(P4,LOW);   // Set CLOSED LOW
      pcf8574.digitalWrite(P5,LOW);   // Set OPEN LOW
    }
  }
 
  client.publish("garage/openStatus", msg_door);

}

// ============================ READ LDR SENSOR ======================
//
// Description:
//
// ====================================================================
void readLDRSensor(int &ldrSensorValue) {

    ldrSensorValue = analogRead(ldrPin);
    if (ldrSensorValue > lightOnLimit)
      lightStatus = true;
    else
      lightStatus = false;

    char msg_light[50];
    dtostrf(ldrSensorValue,4,1,msg_light);
    client.publish("garage/lightlevel", msg_light);
    Serial.print(msg_light);
    Serial.print("\t");
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
      Serial.println(pirMotionDetected);
      if (pirMotionDetected == HIGH) {
        client.publish("garage/motionActive", "ON");
        Serial.println("Motion Detected");
        pcf8574.digitalWrite(P6,HIGH);
      } else {
        client.publish("garage/motionActive", "OFF");
        Serial.println("No Motion Detected");
        pcf8574.digitalWrite(P6,LOW);
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

    //dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
    dtostrf(DS18B20_temp,4,1,msg_gt);
    client.publish("garage/garagetemp", msg_gt);
    Serial.print(msg_gt);
    Serial.print("\t");
}

// ============================ READ DHT SENSOR =======================
//
// Description:
//
// ====================================================================
void readDHT11Sensor(char *msg_ot, char *msg_oh) {
    
    if (failedCount < failedCountLimit){
      t = dht.readTemperature();
      h = dht.readHumidity();
      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t) ) {
        Serial.println("Failed to read from DHT sensor!");
        failedCount++;
      } else {
        failedCount = 0;
      }
    }
    
    dtostrf(t,4,1,msg_ot);
    client.publish("garage/outsidetemp", msg_ot);
    Serial.print(msg_ot);
    Serial.print("\t");
    
    dtostrf(h,4,1,msg_oh);
    client.publish("garage/outsidehumidity", msg_oh);
    Serial.print(msg_oh);
    Serial.print("\t");

}


// ============================ READ PCF8574 SENSOR =======================
//
// Description:
//
// ====================================================================
void readPCF8574Sensor(PCF8574::DigitalInput &di) {
  PCF8574::DigitalInput returnValue = pcf8574.digitalReadAll();
  di = returnValue;
  // This doesn't work because di is being set to the address of returnValue which doesn't exist when the function exits!
}

// ====================== Display Method =================
// 
// Method that is called to display the data on the OLED
// 
// ===================================================================
void displayScreen(char *msg_gt, char* msg_oh, char* msg_ot, char* msg_door, int ldrSensorValue) {
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
      //showGarageTemperature(); 
      break;
    case OUTSIDEHUMIDITY: 
      showOutsideHumidity(msg_oh);
      //showOutsideHumidity();
      break;
    case OUTSIDETEMPERATURE: 
      showOutsideTemperature(msg_ot); 
      //showOutsideTemperature(); 
      break;
    case DOORSTATUS:
      showDoorStatus(msg_door);
      break;
    case LIGHTSTATUS:
      showLightStatus(ldrSensorValue);
      //showLightStatus();
      break;
    case TIME:
      showTime();
      break;
    default:
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


// ============================ SHOW WELCOME DISPLAY =======================
//
// Description:
//
// ===========================================================================
void showInit()
{
  clearDisplay();
       
  display.println("SENSOR");
  display.println("INIT"); 

  display.display();
}


// ============================ SHOW OUTSIDE TEMPERATURE DISPLAY ============
//
// Description:
//
// ===========================================================================
void showOutsideTemperature(char *msg_ot)
//void showOutsideTemperature()
{
  clearDisplay();
       
  display.println("OUTSIDE:");
  display.print(msg_ot); // DHT11 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}


// ============================ SHOW OUTSIDE HUMIDITY DISPLAY ================
//
// Description:
//
// ===========================================================================
void showOutsideHumidity(char *msg_oh)
//void showOutsideHumidity()
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
void showGarageTemperature(char *msg_gt)
//void showGarageTemperature()
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
void showDoorStatus(char *msg_door)
{
  clearDisplay();
       
  display.println("DOOR");
  display.println(msg_door);
  Serial.print("Door Status: ");
  Serial.println(msg_door);
  /*if (openStatus == true)
    display.println("OPEN");
  else if (closedStatus == true)
    display.println("CLOSED");
  else
    display.println("IN TRANSIT");*/

  display.display();
}


// ============================ SHOW LIGHT STATUS DISPLAY ===================
//
// Description:
//
// ===========================================================================
void showLightStatus(int ldrSensorValue)
//void showLightStatus()
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
void CheckGarageDoorStatus() {
  
  if (/*closedStatus*/false) {    // Door is closed, nothing else to do
    doorOpenTime = millis();
    return;
  }

  if (pirMotionDetected) {
      Serial.println("Motion Detected!  Reset timer!");
      doorOpenTime = millis();
      return;
    }
  long garageDoorTimeElapsed = millis() - doorOpenTime;
  Serial.print("Garage Door Time Elapsed: "); Serial.println(garageDoorTimeElapsed);
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
  digitalWrite(relayPin, LOW);   // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because it is acive low on the ESP-01)
  delay(1000);
  digitalWrite(relayPin, HIGH);  // Turn the LED off by making the voltage HIGH
  Serial.println("Send Open/Close Garage Door Command!"); 
}

// ============================ SETUP ==================================
//
// Description:
//
// ====================================================================
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  
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


  Serial.println("SETUP COMPLETED SUCCESSFULLY!");
}

// ============================ LOOP ==================================
//
// Description:
//
// ====================================================================
void loop() {

char msg_gt[50];
char msg_oh[50];
char msg_ot[50];
char msg_door[50];
char msg_motion[50];
int ldrSensorValue;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  //ArduinoOTA.handle();

  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
    }
  }
  
  if (millis() - lastReadTime > sensorReadInterval) {
    lastReadTime = now();
    Serial.println();

    if (pcf8574Configured == true) {
      readDoorSensor(msg_door);
    }

    readLDRSensor(ldrSensorValue);

    readPIRSensor();

    readDS18B20Sensor(msg_gt);

    readDHT11Sensor(msg_ot, msg_oh);

    CheckGarageDoorStatus();

  }
  
  displayScreen(msg_gt, msg_oh, msg_ot, msg_door, ldrSensorValue);
  
}
