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
bool connected = false;
// Update these with values suitable for your network.
const char* ssid = "RedBear";
const char* password = "VLgregRy4h";
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
// These two values must be specified in the Adafruit_SSD1306-master.h file
//#define SSD1306_I2C_ADDRESS 0x76
//#define SSD1306_128_32
// ============================== END OLED ===========================


// ============================= SCREEN SETUP ======================
int screen = 0;    
int screenMax = 5;
bool screenChanged = true;   // initially we have a new screen,  by definition 
// defines of the screens to show
#define GARAGETEMPERATURE     0
#define HUMIDITY              1
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
// Data wire is plugged into pin D8 on the Arduino
#define ONE_WIRE_BUS D4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
double DS18B20_temp = 0;
// ============================== END DS18B20 ======================

// ============================== MQTT Setup =======================
#include<stdlib.h>
WiFiClient espClient;
PubSubClient client(espClient);
const char* mqtt_server = "192.168.0.101";
long lastMsg = 0;
char msg_t[50];
char msg_h[50];
char msg_ot[50];
int value = 0;
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
int ldrSensorValue = 0;
bool lightStatus = false;

int pirPin = D3;
bool pirMotionDetected = false;
// ============================== END GARAGE ===========================


// ============================== TIME Setup ======================
#include <TimeLib.h>
#include <WiFiUdp.h>
static const char ntpServerName[] = "us.pool.ntp.org";
//const int timeZone = 1;     // Central European Time
const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);
time_t prevDisplay = 0; // when the digital clock was displayed

// ============================== END TIME ===========================



void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  
  // --------- setup OLED  --------- 
  setupOLED();

  // --------- setup DS18B20 Temp Sensor --------- 
  setupTempSensor();

  // --------- setup DHT  --------- 
  setupDHT();

  // --------- setup PIR  --------- 
  setupPIRSensor();
  
  // --------- setup Output Relay  --------- 
  setupGarageSensors();

  // --------- setup Wifi Connection  --------- 
  setupWifi();

  // --------- setup MQTT  --------- 
  setupMQTT();

  // --------- setup NTP  --------- 
  setupNTP();

  // --------- setup OTA  --------- 
  //setupOTA();
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void setupOLED() {
    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen internally, this will display the splashscreen.
  showWelcome();
  // init done
  Serial.println("OLED Display Initialized!");
}

// ============================ SETUP TEMP SENSOR =====================
//
// Description:
//
// ====================================================================
void setupTempSensor() {
  sensors.begin();
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

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA); // <<< Station
  WiFi.begin(ssid, password);

  // Try to connect 10 times with 500 msec inbetween each attempt
  for (int i =0; i < 10; i++)
  {
    if (WiFi.status() != WL_CONNECTED) {
      delay(500);
    } else {
      connected = true;
      continue;
    }
  }

  if (connected)
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }

}

// ========================= SETUP GARAGE SENSOR =======================
//
// Description:
//
// =====================================================================
void setupGarageSensors() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Set high to disable
  pinMode(openStatusPin, INPUT);
  pinMode(closedStatusPin, INPUT);
 
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
  setSyncInterval(300);
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

// ============================ MQTT CALLBACK==========================
//
// Description:
//
// ====================================================================
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic,"garage/openDoor")==0){
    // whatever you want for this topic
      // Switch on the LED if an 1 was received as first character
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
      Serial.print("failed, rc=");
      delay(5000);
    }
  }
}


// ============================ LOOP ==================================
//
// Description:
//
// ====================================================================
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  ArduinoOTA.handle();

  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) { //update the display only if time has changed
      prevDisplay = now();
    }
  }
  
  long now = millis();
  if (now - lastMsg > 1000) {
    lastMsg = now;

    readDoorSensor();

    readLDRSensor();

    readPIRSensor();

    readDS18B20Sensor();

    readDHT11Sensor();
                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    publishEnvironmentData();

    CheckGarageDoorStatus();

    displayScreen();
  }

  
}

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
void readLDRSensor() {
    ldrSensorValue = analogRead(ldrPin);
    if (ldrSensorValue > 600)
      lightStatus = true;
    else
      lightStatus = false;

    char msg_light[50];
    dtostrf(ldrSensorValue,4,1,msg_light);
    client.publish("garage/lightlevel", msg_light);
    Serial.print(ldrSensorValue);
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

// ============================ READ TEMP SENSOR ======================
//
// Description:
//
// ====================================================================
void readDS18B20Sensor() {
        // read DS18B20 temperature sensor at index 0 (first sensor)
    sensors.requestTemperatures();  
    DS18B20_temp = sensors.getTempCByIndex(0);
}

// ============================ READ DHT SENSOR =======================
//
// Description:
//
// ====================================================================
void readDHT11Sensor() {
    //int rtn = dht.read(dhtPin);
    t = dht.readTemperature();
    h = dht.readHumidity();
    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t) ) {
      Serial.println("Failed to read from DHT sensor!");
    }
}

// ============================ PUBLISH ENVIRONMENTAL DATA =================
//
// Description:
//
// =========================================================================
void publishEnvironmentData() {
    dtostrf(t,4,1,msg_t);
    client.publish("garage/outsidetemp", msg_t);
    Serial.print(msg_t);
    Serial.print("\t");
    
    dtostrf(h,4,1,msg_h);
    client.publish("garage/outsidehumidity", msg_h);
    Serial.print(msg_h);
    Serial.print("\t");
    
    //dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
    dtostrf(DS18B20_temp,4,1,msg_ot);
    client.publish("garage/garagetemp", msg_ot);
    Serial.println(msg_ot);
}

// ====================== Display Method =================
// 
// Method that is called to display the Temperature and Humidity data on the OLED
// 
// ===================================================================
void displayScreen() {
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
      showTemperature(10); 
      break;
    case HUMIDITY: 
      showHumidity(20);
      break;
    case OUTSIDETEMPERATURE: 
      showGarageTemperature(30); 
      break;
    case DOORSTATUS:
      showDoorStatus(40);
      break;
    case LIGHTSTATUS:
      showLightStatus(50);
      break;
    case TIME:
      showTime(60);
      break;
    default:
      // cannot happen -> showError() ?
      break;
    }
  }
}


// ============================ CLEAR DISPLAY ===============================
//
// Description:
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
void showWelcome()
{
  clearDisplay();
       
  display.println("SENSOR");
  display.println("INIT"); 

  display.display();

  delay(2000);
}


// ============================ SHOW OUTSIDE TEMPERATURE DISPLAY ============
//
// Description:
//
// ===========================================================================
void showTemperature(int T)
{
  clearDisplay();
       
  display.println("OUTSIDE:");
  display.print(t,1); // DHT11 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}


// ============================ SHOW OUTSIDE HUMIDITY DISPLAY ================
//
// Description:
//
// ===========================================================================
void showHumidity(int H)
{
  clearDisplay();
       
  display.println("HUMIDITY:");
  display.print(h,1);
  display.println(" %");

  display.display();
}


// ============================ SHOW GARAGE TEMPERATURE DISPLAY ==============
//
// Description:
//
// ===========================================================================
void showGarageTemperature(int T)
{
  clearDisplay();
       
  display.println("GARAGE:");
  display.print(msg_ot); // DS18B20 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}


// ============================ SHOW GARAGE DOOR STATUS DISPLAY ===============
//
// Description:
//
// ===========================================================================
void showDoorStatus(int T)
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
void showLightStatus(int T)
{
  clearDisplay();
       
  display.print("LIGHT");
  display.print(" ");
  display.println(ldrSensorValue);
  if (lightStatus == true)
    display.println("ON");
  else
    display.println("OFF");

  display.display();
}

// ============================ SHOW TIME DISPLAY ===========================
//
// Description:
//
// ===========================================================================
void showTime(int T)
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
void showOpenCloseGarageDoor(int T)
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
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 500) {    // was 1500
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
void sendNTPpacket(IPAddress &address)
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
  showOpenCloseGarageDoor(15);
  digitalWrite(relayPin, LOW);   // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because it is acive low on the ESP-01)
  delay(1000);
  digitalWrite(relayPin, HIGH);  // Turn the LED off by making the voltage HIGH
  Serial.println("Send Open/Close Garage Door Command!"); 
}

