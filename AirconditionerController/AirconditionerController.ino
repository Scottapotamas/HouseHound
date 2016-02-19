//designed for aircon controller board.
//BMP280, HDC1000 provide basic weather data
//SDC1306 2.4in oled for display
//binary style output for pump control
//1.2-3.2V for fan control
//WS2812 for status
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
ESP8266WiFiMulti WiFiMulti;

#define USE_SERIAL Serial


// Display (i2c)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1  //board has reset line to screen on pin2
Adafruit_SSD1306 display(OLED_RESET);

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

void initSensors() {
  if (!hdc.begin()) {
    Serial.println("Couldn't find HDC1000 sensor!");
  }

  if (!bmp.begin()) {  
    Serial.println("Could not find BMP280 sensor!");
  }
}

void initPins() {

    //fan and water control goes here

}

void setup() {

    USE_SERIAL.begin(115200);
   // USE_SERIAL.setDebugOutput(true);
    USE_SERIAL.println();

    // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3C (for the 128x32)
    display.clearDisplay();
    
    displayBootScreen(" Starting Sensors   ");


    initSensors();

    displayBootScreen("  Initialising I/O  ");
    initPins();
    strip.begin();

    displayBootScreen(" Connecting to Wifi ");

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    WiFiMulti.addAP("Raptor Jnr", "bicycl35ar3funt0rid3");
 
    displayBootScreen("        Done        ");
    delay(1000);
}

void loop() {
    
    display.clearDisplay();


    getLocalSensors();
    getExternalSensors();       //TODO get data from OPENHAB
    getInternetSensors();       //TODO get weather data from online

    // displayWeatherInfo();
    displayAirconInfo();        //TODO
    displayStatusLED();         //TODO


    //TODO: Decouple draw with loop, state machine to wait between draws


    display.display();

    delay(4000);
}

void displayBootScreen(String subText) {
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0,16);
    display.print("  Booting  ");

    if(subText != "") {
        display.setTextSize(1);
        display.setCursor(0,50);
        display.print(subText);
    }

    display.display();
}


void displayWeatherInfo() 
{
    display.setTextColor(WHITE);

    drawWeatherDataBlock(32,1,2, "Indoor", tempLocal, humidityLocal, baroLocal);
    drawWeatherDataBlock(32,40,1, "Outdoor", tempLocal2, humidityLocal, baroLocal);
    
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
        display.setCursor(x+28, y+16);
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
    display.setCursor(colPos,0);
    display.print("%-20s", "initialization...");

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(colPos,10);
    display.println("MODE");
    
    display.setCursor(colPos,8);
    display.println("FAN");

    display.setCursor(colPos,16);
    display.println("WATER");
    
    display.setCursor(colPos,48);
    display.println("TIME");
}

void displayStatusLED() 
{
    strip.setPixelColor(1, strip.Color(150,150,150)); 

    strip.show(); // This sends the updated pixel color to the hardware.
}


void getLocalSensors() 
{
    tempLocal = hdc.readTemperature(); 
    tempLocal2 = bmp.readTemperature();
    humidityLocal = hdc.readHumidity();  
    baroLocal = bmp.readPressure() / 100; 
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


void getWebData() {

// wait for WiFi connection
if((WiFiMulti.run() == WL_CONNECTED)) {

    HTTPClient http;

    USE_SERIAL.print("[HTTP] begin...\n");
    // configure traged server and url
    //http.begin("192.168.1.12", 443, "/test.html", true, "7a 9c f4 db 40 d3 62 5a 6e 21 bc 5c cc 66 c8 3e a1 45 59 38"); //HTTPS
    http.begin("192.168.1.68", 80, "/"); //HTTPS

    USE_SERIAL.print("[HTTP] GET...\n");
    // start connection and send HTTP header
    int httpCode = http.GET();
    if(httpCode) {
        // HTTP header has been send and Server response header has been handled
        USE_SERIAL.printf("[HTTP] GET... code: %d\n", httpCode);

        // file found at server
        if(httpCode == 200) {
            String payload = http.getString();
            USE_SERIAL.println(payload);

            display.setTextSize(2);
            display.setTextColor(WHITE);
            display.setCursor(0,48);
            display.println("Get good!");
            //display.println(payload);
            display.display();
            
        }
    } else {
        USE_SERIAL.print("[HTTP] GET... failed, no connection or no HTTP server\n");
        
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(0,48);
        display.println("Get failed");
        display.display();
    }
}

}