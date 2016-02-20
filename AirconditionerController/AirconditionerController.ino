//designed for aircon controller board.
//BMP280, HDC1000 provide basic weather data
//SDC1306 2.4in oled for display
//binary style output for pump control
//1.2-3.2V for fan control
//WS2812 for status
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//ESP Wifi Stuff
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266SSDP.h>
#include <ESP8266mDNS.h>

#define wifi_ssid "Raptor Jnr"
#define wifi_password "bicycl35ar3funt0rid3"

WiFiClient espClient;
ESP8266WebServer HTTP(80);

int wifiAttempts = 0;       //track connection attempts to graciously fallback to local only operation

// MQTT
#include <PubSubClient.h>       
PubSubClient client(espClient);

#define mqtt_server "YOUR_MQTT_SERVER_HOST"
#define mqtt_user "your_username"
#define mqtt_password "your_password"

#define humidity_topic "sensor/humidity"
#define temperature_topic "sensor/temperature"


// Display (i2c)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1  //board has reset line to screen on pin2
Adafruit_SSD1306 display(OLED_RESET);

const unsigned char PROGMEM WaterIcon [] = {
0x10, 0x38, 0x38, 0x7C, 0xFE, 0xFE, 0xFE, 0x7C, 
};

const unsigned char PROGMEM FanIcon [] = {
0xC3, 0xE7, 0x6E, 0x20, 0x04, 0x76, 0xE7, 0xC3, 
};

//Temp/Humidity Sensor (i2c)
#include "Adafruit_HDC1000.h"
Adafruit_HDC1000 hdc = Adafruit_HDC1000();

//Barometric Pressure Sensor (i2c)
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

//WS2812 Status LED (Pin 15?)
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, 15, NEO_GRB + NEO_KHZ800);

#define PUMP_PIN 12
#define FAN_PIN 13

#define BUTTON_1    14
#define BUTTON_2    15
#define BUTTON_3    0

float tempLocal     = 0;
float tempLocal2    = 0;
float humidityLocal = 0;
float baroLocal     = 0;

float tempExternal      = 0;
float humidityExternal  = 0;
float baroExternal      = 0;

int fanSpeed        = 0;   //percentage value
int waterPumpState  = 0;
int controlMode     = 0;

const int colPos = 64; //x position of second column start


void initSensors() 
{
    displayBootScreen("  Starting Sensors  ");

    if (!hdc.begin()) 
    {
        Serial.println("Couldn't find HDC1000 sensor!");
    }

    if (!bmp.begin()) 
    {  
        Serial.println("Could not find BMP280 sensor!");
    }
}

void initPins() {
    displayBootScreen("  Initialising I/O  ");
  
    pinMode(PUMP_PIN,OUTPUT);
    digitalWrite(PUMP_PIN, 0);  //set fan speed to 0

    pinMode(FAN_PIN, OUTPUT);
    analogWriteFreq(1000);      //set PWM freq to 1kHz
    analogWriteRange(1023);     //10 bit pwm output
    analogWrite(FAN_PIN, 0);  //set fan speed to 0
}

void setupHTTP() {

    displayBootScreen(" Starting Web Server");

    HTTP.on("/", HTTP_GET, [](){
      HTTP.send(200, "text/plain", "Hello World!");
    });

    HTTP.on("/index.html", HTTP_GET, [](){
      HTTP.send(200, "text/plain", "Index goes here!");
    });

    HTTP.on("/description.xml", HTTP_GET, [](){
      SSDP.schema(HTTP.client());
    });

    HTTP.begin();
}

void setupMDNS() 
{
    displayBootScreen(" Starting mDNS");

    if (!MDNS.begin("brivisevap")) 
    {
        displayBootScreen("mDNS Error! Retrying");
        delay(2000);
        return setupMDNS();       //woo fake while-loop with recursion!
    }
    
    displayBootScreen("    mDNS started   ");
    MDNS.addService("http", "tcp", 80);
}

void setupSSDP()  //shared service discovery protocol
{
    displayBootScreen("Starting SSDP Server");

    SSDP.setSchemaURL("description.xml");
    SSDP.setHTTPPort(80);
    SSDP.setName("Brivis Evaporative Aircon");
    SSDP.setSerialNumber("001788102201");
    SSDP.setURL("index.html");
    SSDP.setModelName("Brivis Wifi Controller");
    SSDP.setModelNumber("929000226503");
    SSDP.setModelURL("http://www.google.com");
    SSDP.setManufacturer("Scott Rapson");
    SSDP.setManufacturerURL("http://www.26oclock.com");
    SSDP.begin();

    //TODO Replace these with something reasonable for the Brivis unit

    displayBootScreen("SSDP Server Started");
}

void setupWiFi() 
{
    displayBootScreen("  Connect to Wifi ");
    display.setTextSize(1);
    display.setCursor(20,55);
    display.print("SSID: ");
    display.print(wifi_ssid);
    display.display();
    delay(500);

    if(wifiAttempts == 0)   //TODO do something with timeouts here
    {
        WiFi.begin(wifi_ssid, wifi_password);
    }


    if(WiFi.status() == WL_CONNECTED) {
        //display connection info on screen
        displayBootScreen("  Connected at IP ");
        display.setTextSize(1);
        display.setCursor(20,55);
        display.print(WiFi.localIP());
        display.display();
        //allow user a quick moment to see the IP if they are too lazy/incompetent to do infrastructure level DHCP allocations
        delay(1000);    
        return;
    } 
    else 
    {
        wifiAttempts++;
        displayBootScreen("  Error! Retrying  ");
        delay(2000);
        return setupWiFi();       //woo fake while-loop with recursion!
    }

}

void setupMQTT()
{
    displayBootScreen("  Initiating MQTT   ");
    client.setServer(mqtt_server, 1883);
}

void setup() {

    Serial.begin(115200);
   // USE_SERIAL.setDebugOutput(true);
    Serial.println("");

    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3C (for the 128x32)
    display.clearDisplay();
    
    initSensors();
    initPins();

    strip.begin();

    setupWiFi();    //connect to the existing network
    setupHTTP();    //web server
    setupMDNS();    //for brivisevap.local through bonjour
    setupSSDP();    
    //setupMQTT();

    //done!
}

void loop() {
    
    display.clearDisplay();
    
    HTTP.handleClient();        //SSDP etc
    //mqttLoop();                 //MQTT code is self contained, this runs the loop level code it requires

    getLocalSensors();
    getExternalSensors();       //TODO get data from OPENHAB
    getInternetSensors();       //TODO get weather data from online

    displayWeatherInfo();
    displayAirconInfo();        //TODO
    displayStatusLED();         //TODO


    //TODO: Decouple draw with loop, state machine to wait between draws


    display.display();      //draw once per loop. We don't do this per function to prevent tearing style updates

    delay(4000);
}

// ------ OLED DISPLAY FUNCTIONS  ------------


void displayBootScreen(String subText) {
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,16);
    display.print("  Booting  ");

    if(subText != "") {
        display.setTextSize(1);
        display.setCursor(5,45);
        display.print(subText);
    }

    display.display();
}


void displayWeatherInfo() 
{
    display.setTextColor(WHITE);

    drawWeatherDataBlock(3,2,2, "Indoor", tempLocal, humidityLocal, baroLocal);
    drawWeatherDataBlock(3,41,1, "Outdoor", tempLocal2, humidityLocal, baroLocal);
    
    //TODO draw internet weather
    //drawWeatherDataBlock(32,40,1, "Internet", tempLocal2, humidityLocal, baroLocal);
}

void drawWeatherDataBlock(int x, int y, int dataLevel, String title, float temp, float humidity, float baro) 
{
    display.setTextColor(WHITE);

    //title text
    display.setTextSize(1);
    display.setCursor(x,y);
    display.print(title);
    
    //temperature text
    display.setTextSize(2);
    display.setCursor(x,y+9);
    display.print(tempLocal, 0);    //no decimal places
    
    display.setTextSize(1);
    display.setCursor(x+24,y+7);
    display.print((char)223);       //degrees symbol
    
    //humidity text
    if(dataLevel >= 1) 
    {
        display.setTextSize(1);
        display.setCursor(x+29, y+16);
        display.print(humidityLocal, 0); display.print("%");
    }

    //barometric pressure text
    if(dataLevel >= 2) 
    {
        display.setCursor(x,y+26);
        display.print(baroLocal, 0); display.print(" hPa");
    }

}

void displayAirconInfo() 
{
    display.setTextColor(WHITE);
    display.setTextSize(2);
    
    int x = 75;
    int y = 3;

    switch(controlMode)
    {
        case 0:     //uninitalised
            display.setCursor(x,y);
            display.println("WAIT");
            controlMode++;
        break;

        case 1:     //off
            display.setCursor(x,y);
            display.println(" OFF");
        break;

        case 2:     //manual fan
            display.setCursor(x,y);
            display.println(" FAN");
        break;

        case 3:     //manual fan+water
            display.setCursor(x,y);
            display.println("COOL");
        break;

        case 4:     //auto
            display.setCursor(x,y);
            display.println("AUTO");
        break;

        case 5:     //web controlled
            display.setCursor(x,y);
            display.println(" WEB");
        break;        

        default:

        break;
    }


    display.setTextSize(1);
    x += 10;
    y += 24;

    if(fanSpeed < 1)    //fan is off
    {
        display.drawBitmap(x, y, FanIcon, 8, 8, WHITE);
        display.setCursor(x+10,y);
        display.println("OFF ");
    }
    else                //fan is on
    {
        display.drawBitmap(x, y, FanIcon, 8, 8, WHITE);
        display.setCursor(x+10,y);
        display.print(fanSpeed);    display.print("%");
    }

    y += 10;
    if(waterPumpState == 0) //pump is off
    {
        display.drawBitmap(x, y, WaterIcon, 8, 8, WHITE);
        display.setCursor(x+10,y);
        display.println("OFF");
    }
    else        //pump is on
    {
        display.drawBitmap(x, y, WaterIcon, 8, 8, WHITE);
        display.setCursor(x+10,y);
        display.println(" ON");
    }

    x -= 2;
    y += 19;
    display.setCursor(x,y);
    display.println("12:34");

}

void displayStatusLED() 
{
    strip.setPixelColor(1, strip.Color(150,150,150)); 

    strip.show(); // This sends the updated pixel color to the hardware.
}

// ------ CONTROL OF HARDWARE  ------------

void setWaterPump(bool state) 
{
    digitalWrite(PUMP_PIN, state);
}

void setFanOutput(int percentage) 
{

    if(controlMode < 2)
    {
        //shouldn't run the fan
        percentage = 0;
    }

    int output = map(percentage, 0, 100, 0, 1023);  //map percentage to 10bit output for PWM

    analogWrite(PUMP_PIN, output);
}

// ------ AQUISITION OF SENSOR DATA  ------------

void getLocalSensors() 
{
    tempLocal       = hdc.readTemperature(); 
    tempLocal2      = bmp.readTemperature();
    humidityLocal   = hdc.readHumidity();  
    baroLocal       = bmp.readPressure() / 100;   //convert to hPa 
}

void getExternalSensors() {
    //request data from OpenHAB

}

void getInternetSensors() {

    //TODO ask for published weather data
}

void getInternetTime() {

    //TODO ask for NTP time to keep reference with
}




// ------  MQTT CONNECTION HANDLING AND MESSAGING ------------


void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
    // if (client.connect("ESP8266Client")) {
    if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}


long lastMsg = 0;
float temp = 0.0;
float hum = 0.0;
float diff = 1.0;

void mqttLoop() 
{
    if (!client.connected()) {
        mqttReconnect();
    }
    client.loop();


    long now = millis();
    if (now - lastMsg > 1000) 
    {
        lastMsg = now;

        float newTemp = hdc.readTemperature();
        float newHum = hdc.readHumidity();

        if (checkBound(newTemp, temp, diff)) 
        {
          temp = newTemp;
          Serial.print("New temperature:");
          Serial.println(String(temp).c_str());
          client.publish(temperature_topic, String(temp).c_str(), true);
        }

        if (checkBound(newHum, hum, diff)) 
        {
          hum = newHum;
          Serial.print("New humidity:");
          Serial.println(String(hum).c_str());
          client.publish(humidity_topic, String(hum).c_str(), true);
        }
    }
}


