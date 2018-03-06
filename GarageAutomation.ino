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
  D3 = 0 (DHT11)
  D4 = 2
  D5 = 14
  D6 = 12
  D7 = 13
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

// ============================== END GARAGE ===========================



void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  
  // --------- setup OLED  --------- 
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen internally, this will display the splashscreen.
  display.display();
  delay(1000);
  // Clear the buffer.
  display.clearDisplay();
  // init done
  Serial.println("OLED Display Initialized!");

  // --------- setup Output Relay  --------- 
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH); // Set high to disable

  // --------- setup Wifi Connection  --------- 
  setup_wifi();

  // --------- setup OLED  --------- 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // --------- setup DS18B20 Temp Sensor --------- 
  sensors.begin();
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA); // <<< Station
  WiFi.begin(ssid, password);

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

    if (!connected) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setCursor(0,0);
  
    display.println("Connection");
    display.println("Error!");
  
    display.display();
  }
  
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

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
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      delay(5000);
    }
  }
}


void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;

    // read DS18B20 temperature sensor at index 0 (first sensor)
    sensors.requestTemperatures();  
    DS18B20_temp = sensors.getTempCByIndex(0);
    
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
    //snprintf (msg_t, 25, "%f", DS18B20_temp);
    //Serial.print("Publish message: ");
    //Serial.println(msg);
    dtostrf(t,4,1,msg_t);
    client.publish("garage/temp", msg_t);
    Serial.print(msg_t);
    Serial.print("\t");
    
    dtostrf(h,4,1,msg_h);
    client.publish("garage/humidity", msg_h);
    Serial.print(msg_h);
    Serial.print("\t");
    
    //snprintf (msg_ot, 25, "%2.1f", DS18B20_temp);
    //dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);
    dtostrf(DS18B20_temp,4,1,msg_ot);
    client.publish("garage/outsidetemp", msg_ot);
    Serial.println(msg_ot);

    displayState1();
  }
}


// ====================== State 1 (Secondary) Display Method =================
// 
// Method that is called to display the Temperature and Humidity data on the OLED
// 
// ===================================================================
void displayState1(){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0,0);

  /*display.print("T:");
  display.print(t,1); // DHT11 Temperature
  display.println(" C");

  display.print("H:");
  display.print(h);
  display.println(" %");*/

  display.print("OUT:");
  //display.print(DS18B20_temp,1);
  display.print(msg_ot);
  display.println(" C");

  display.display();
}


void printDouble( double val, byte precision){
  // prints val with number of decimal places determine by precision
  // precision is a number from 0 to 6 indicating the desired decimial places
  // example: lcdPrintDouble( 3.1415, 2); // prints 3.14 (two decimal places)
  
  int intPart;
  int decPart;
  char msgBuffer[50];
  /*if(val < 0.0)
  {
   Serial.print('-');
   val = -val;
  }*/
  
  //Serial.print (int(val));  //prints the int part
  intPart = int(val);
  
  if( precision > 0) {
   Serial.print("."); // print the decimal point
   unsigned long frac;
   unsigned long mult = 1;
   byte padding = precision -1;
   while(precision--)
  mult *=10;
  
   if(val >= 0)
  frac = (val - int(val)) * mult;
   else
  frac = (int(val)- val ) * mult;
   unsigned long frac1 = frac;
   while( frac1 /= 10 )
  padding--;
   while(  padding--)
  Serial.print("0");
   Serial.print(frac,DEC) ;
  }
  snprintf (msgBuffer, 25, "%d.%d", intPart, decPart);
}
