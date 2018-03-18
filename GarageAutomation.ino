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
  D1 = 5 (OLED)
  D2 = 4 (OLED)
  D3 = 0 (PIR)
  D4 = 2 (DS18B20)
  D5 = 14 (Door Open)
  D6 = 12 (DHT11)
  D7 = 13 (Door Closed)
  D8 = 15 (NOT USED) - must not be used with pullup resistor or logic will not download
  A0 = A0 (LDR)
*/

// ============================== WIFI SETUP ===========================
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
// Update these with values suitable for your network.
const char* ssid = "RedBear";
const char* password = "VLgregRy4h";
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Try to replace with DS18B20 MAC
// ============================== END WIFI ===========================


// ============================== OTA SETUP ===========================
#include <ArduinoOTA.h>
// ============================== END OTA ===========================

// ============================= OLED DISPLAY SETUP ======================
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(OLED_RESET);
// This value must be specified in the Adafruit_SSD1306-master.h file
//#define SSD1306_128_32
// ============================== END OLED ===========================


// ============================= SCREEN SETUP ======================
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
// ============================== END SCREEN SETUP ===========================


// ============================== DHT11 ============================== 
#include <DHT.h>
#define DHTTYPE  DHT21
int dhtPin=D6;
DHT dht(dhtPin, DHTTYPE);
float t = 0;
float h = 0;
// ============================== END DHT11 ===========================


// ============================== DS18B20 Setup ======================
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into pin D4 on the Arduino
#define ONE_WIRE_BUS D4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
double DS18B20_temp = 0;
// ============================== END DS18B20 ======================

// ============================== MQTT Setup =======================
//#include<stdlib.h>
WiFiClient espClient;
PubSubClient client(espClient);
const char* mqtt_server = "192.168.0.101";
//char msg_t[50];
//char msg_h[50];
//char msg_ot[50];
// ============================== END MQTT ===========================


// ============================== GARAGE Setup ======================
int relayPin = D0;

int openStatusPin = D5;
int closedStatusPin = D7;
bool openStatus = false;
bool closedStatus = false;
long doorOpenTime = 0;
int garageDoorOpenTimeLimit = 15*60*1000;

int ldrPin = A0;
bool lightStatus = false;
const int lightOnLimit = 400;
int ldrSensorValue;

int pirPin = D3;
bool pirMotionDetected = false;

long lastReadTime = 0;
int sensorReadInterval = 1000;
// ============================== END GARAGE ===========================


// ============================== TIME Setup ======================
#include <TimeLib.h>
#include <WiFiUdp.h>
static const char ntpServerName[] = "ca.pool.ntp.org";
//const int timeZone = 1;     // Central European Time
const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

/*time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);*/
time_t prevDisplay = 0; // when the digital clock was displayed

// ============================== END TIME ===========================

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
  pinMode(openStatusPin, INPUT);
  pinMode(closedStatusPin, INPUT);
  pinMode(pirPin, INPUT);
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

// ============================ SETUP NTP ============================
//
// Description:
//
// ====================================================================
void setupNTP() {
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);             // 300 seconds = 5 min
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
void readDoorSensor() {

    // read door status
    if (digitalRead(openStatusPin) == HIGH) {
      openStatus = true;
      client.publish("garage/openStatus", "TRUE");
    } else {
      openStatus = false;
      client.publish("garage/openStatus", "FALSE");
    }
    
    if (digitalRead(closedStatusPin) == HIGH) {
      closedStatus = true;
      client.publish("garage/closedStatus", "TRUE");
    } else {
      closedStatus = false;
      client.publish("garage/closedStatus", "FALSE");
    }
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
    pirMotionDetected = digitalRead(pirPin);
    char msg_motion[50];
    dtostrf(pirMotionDetected,4,1,msg_motion);
    client.publish("garage/motionactive", msg_motion);
    Serial.print(msg_motion);
    Serial.print("\t");
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
    //int rtn = dht.read(dhtPin);
    t = dht.readTemperature();
    h = dht.readHumidity();
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) ) {
      Serial.println("Failed to read from DHT sensor!");
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


// ====================== Display Method =================
// 
// Method that is called to display the data on the OLED
// 
// ===================================================================
void displayScreen(char *msg_gt, char* msg_oh, char* msg_ot, int ldrSensorValue) {
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
      showDoorStatus();
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
void showDoorStatus()
{
  clearDisplay();
       
  display.println("DOOR");
  if (openStatus == true)
    display.println("OPEN");
  else if (closedStatus == true)
    display.println("CLOSED");
  else
    display.println("IN TRANSIT");

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

// ============================ GET NTP TIME ============================
//
// Description:
//
// ======================================================================
time_t getNtpTime()
{
  const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
  byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName); Serial.print(": "); Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP, NTP_PACKET_SIZE, packetBuffer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      int currentTimeZone = timeZone;
      time_t currentTime = secsSince1900 - 2208988800UL + currentTimeZone * SECS_PER_HOUR;
      if (isDST(currentTime))
      {
        currentTimeZone++;
        Serial.println ("DST Active");
      } else {
        Serial.println("DST Not Active");
      }

      if(isDaytime())
        Serial.println("Daytime");
      else
        Serial.println("Nighttime");

      return secsSince1900 - 2208988800UL + currentTimeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// ============================ SEND NTP REQUEST FOR TIME ====================
//
// Description:
// send an NTP request to the time server at the given address
//
// ===========================================================================
void sendNTPpacket(IPAddress &address, const int NTP_PACKET_SIZE, byte packetBuffer[])
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}


// ============================ CHECK GARAGE DOOR STATUS ===============
//
// Description:
//
// ======================================================================
void CheckGarageDoorStatus() {
  if (closedStatus) {    // Door is closed, nothing else to do
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

  // --------- setup NTP  --------- 
  //setupNTP();   // NTP must be setup after Wifi or NodeMCU will crash!

  // --------- setup OTA  --------- 
  //setupOTA();

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
    
    readDoorSensor();

    readLDRSensor(ldrSensorValue);

    readPIRSensor();

    readDS18B20Sensor(msg_gt);

    readDHT11Sensor(msg_ot, msg_oh);

    CheckGarageDoorStatus();

  }
  
  displayScreen(msg_gt, msg_oh, msg_ot, ldrSensorValue);
  
}
