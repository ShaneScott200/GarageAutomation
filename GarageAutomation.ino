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


  NODE MCU Pinout
  D0 = 16 (Relay Output)
  D1 = 5 (OLED)
  D2 = 4 (OLED)
  D3 = 0 (DHT11) now PIR
  D4 = 2
  D5 = 14 (Door Open)
  D6 = 12 (DS18B20)
  D7 = 13 (Door Closed)

  static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;
*/
// ============================== WIFI SETUP ===========================
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
bool connected = false;
// Update these with values suitable for your network.
const char* ssid = "RedBear";
const char* password = "VLgregRy4h";
const char* mqtt_server = "192.168.0.27";
// ============================== END WIFI ===========================


// ============================= OLED Display Setup ======================
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(OLED_RESET);
// These two values must be specified in the Adafruit_SSD1306-master.h file
//#define SSD1306_I2C_ADDRESS 0x76
//#define SSD1306_128_32
// Screen definitions
int screen = 0;    
int screenMax = 4;
bool screenChanged = true;   // initially we have a new screen,  by definition 
// defines of the screens to show
#define GARAGETEMPERATURE 0
#define HUMIDITY    1
#define OUTSIDETEMPERATURE    2
#define DOORSTATUS  3
#define TIME        4
long previousLCDMillis = 0;    // for LCD screen update
long lcdInterval = 4000;
// ============================== END OLED ===========================


// ============================== DHT11 ============================== 
#include <dht11.h>
#define DHTTYPE  DHT11
dht11 dht;
int dhtPin=0;
int t = 0;
int h = 0;
// ============================== END DHT11 ===========================


// ============================== DS18B20 Setup ======================
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into pin D6 on the Arduino
#define ONE_WIRE_BUS 12
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
long lastMsg = 0;
char msg_t[50];
char msg_h[50];
char msg_ot[50];
int value = 0;
// ============================== END MQTT ===========================


// ============================== GARAGE Setup ======================
int relayPin = 16;
int openStatusPin = 14;
bool openStatus = false;
int closedStatusPin = 13;
bool closedStatus = false;
int ldrPin = A0;
int ldrSensorValue = 0;
bool lightStatus = false;
// ============================== END GARAGE ===========================


// ============================== TIME Setup ======================
#include <TimeLib.h>

// ============================== END TIME ===========================



void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  
  // --------- setup OLED  --------- 
  setupOLED();

  // --------- setup DS18B20 Temp Sensor --------- 
  setupTempSensor();
  
  // --------- setup Output Relay  --------- 
  setupGarageSensors();

  // --------- setup Wifi Connection  --------- 
  setupWifi();

  // --------- setup OLED  --------- 
  setupMQTT();


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
        showOpenGarageDoor(10);
        digitalWrite(relayPin, LOW);   // Turn the LED on (Note that LOW is the voltage level
        // but actually the LED is on; this is because it is acive low on the ESP-01)
        delay(1000);
        digitalWrite(relayPin, HIGH);  // Turn the LED off by making the voltage HIGH
      } else {
        digitalWrite(relayPin, HIGH);  // Turn the LED off by making the voltage HIGH
      }
  }

  if (strcmp(topic,"time")==0){
    /*int yr, mon, dy, hr, mn, sec;
    TimeElements tm;
    tm.Second = 0;
    tm.Minute = 16;
    tm.Hour = 19;
    tm.Wday = 3;
    tm.Day = 6;
    tm.Month = 3;
    tm.Year = 47;
    time_t t = makeTime(tm);
    setTime(t);*/
    //setTime(19,24,30,6,3,47);
    // whatever you want for this topic
    //setTime(hr,min,sec,day,month,yr); // Another way to set
    // format of time string: YYYY-MM-DD-HH-MM-SS
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
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

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;

    readDoorSensor();

    readLDRSensor();

    readDS18B20Sensor();

    readDHT11Sensor();
                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    publishEnvironmentData();

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
    if (digitalRead(openStatusPin) == HIGH)
      openStatus = true;
    else 
      openStatus = false;
    if (digitalRead(closedStatusPin) == HIGH)
      closedStatus = true;
    else 
      closedStatus = false;
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
      int rtn = dht.read(dhtPin);
    if (rtn == DHTLIB_OK)
    {
      t = dht.temperature;
      h = dht.humidity;
    }
    else
    {
      Serial.println("Error reading DHT");
    } 
}

// ============================ PUBLISH ENVIRONMENTAL DATA =================
//
// Description:
//
// =========================================================================
void publishEnvironmentData() {
    dtostrf(t,4,1,msg_t);
    client.publish("garage/temp", msg_t);
    Serial.print(msg_t);
    Serial.print("\t");
    
    dtostrf(h,4,1,msg_h);
    client.publish("garage/humidity", msg_h);
    Serial.print(msg_h);
    Serial.print("\t");
    
    //dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
    dtostrf(DS18B20_temp,4,1,msg_ot);
    client.publish("garage/outsidetemp", msg_ot);
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
      showGarageTemperature(40); 
      break;
    case HUMIDITY: 
      showHumidity(50);
      break;
    case OUTSIDETEMPERATURE: 
      showOutsideTemperature(60); 
      break;
    case DOORSTATUS:
      showDoorStatus(70);
      break;
    case TIME:
      showTime(80);
      break;
    default:
      // cannot happen -> showError() ?
      break;
    }
  }
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void clearDisplay() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void showWelcome()
{
  clearDisplay();
       
  display.println("Sensor");
  display.println("Initialising..."); 

  display.display();

  delay(2000);
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void showGarageTemperature(int T)
{
  clearDisplay();
       
  display.println("GARAGE:");
  display.print(t,1); // DHT11 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void showHumidity(int H)
{
  clearDisplay();
       
  display.println("HUMIDITY:");
  display.print(h,1);
  display.println(" %");

  display.display();
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void showOutsideTemperature(int T)
{
  clearDisplay();
       
  display.println("OUTSIDE:");
  display.print(msg_ot); // DS18B20 Temperature
  display.print((char)248);          
  display.println("C");

  display.display();
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
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

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void showLightStatus(int T)
{
  clearDisplay();
       
  display.println("LIGHT");
  if (lightStatus == true)
    display.println("ON");
  else
    display.println("OFF");

  display.display();
}

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
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

// ============================ SETUP OLED ============================
//
// Description:
//
// ====================================================================
void showOpenGarageDoor(int T)
{
  clearDisplay();
       
  display.println("OPEN DOOR");

  display.display();
}
