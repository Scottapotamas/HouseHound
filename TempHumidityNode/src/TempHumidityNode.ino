/*
* Project TempHumidityNode
* Description:  Uses photon and Si7021 temp/humidity sensor.
* Author: Scott Rapson,
          sensor interface code from Sparkfun,
* Date:   July 2017
*/

#include "SparkFun_Photon_Weather_Shield_Library.h"
#include <math.h>

#define SENSOR_SAMPLE_TIME_MS 500
#define PUBLISH_RATE_S 5
#define PUBLISH_RATE_MS (PUBLISH_RATE_S*1000) //seconds into msec

// Timers to maintain sampling and publish throttles
unsigned int timeNextSensorReading;
unsigned int timeNextPublish;

#include "MQTT.h"

void callback(char* topic, byte* payload, unsigned int length);

byte server[] = { 192,168,1,100 };
MQTT client(server, 1883, callback);

// recieve message
void callback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;

    delay(1000);
}

void setup()
{
  client.connect("indoortemp");

  // publish/subscribe
  if (client.isConnected())
  {
      client.publish("debug/indoortemp","Starting Up");
  }

  //setup the sensing and calculation functions
   initializeTempHumidity();

   // Schedule the next sensor reading and publish events
   timeNextSensorReading  = millis() + SENSOR_SAMPLE_TIME_MS;
   timeNextPublish        = millis() + PUBLISH_RATE_MS;
}

void loop()
{
   // Sample sensors that need to be polled
   if( timeNextSensorReading <= millis() )
   {
       captureTempHumidity();

       // Schedule the next sensor reading
       timeNextSensorReading = millis() + SENSOR_SAMPLE_TIME_MS;
   }

   // Publish the data collected to MQTT
   if( timeNextPublish <= millis() )
   {
       // Get the data to be published
       float tempC        = getAndResetTempC();
       float humidityRH   = getAndResetHumidityRH();

       // Publish the data
       //publishToParticle( tempC, humidityRH, pressureKPa, rainMillimeters, windKMH, gustKMH, windDegrees);
       publishToMQTT( tempC, humidityRH);
       //publishToSerial( tempC, humidityRH, pressureKPa, rainMillimeters, windKMH, gustKMH, windDegrees);

       // Schedule the next publish event
       timeNextPublish = millis() + PUBLISH_RATE_MS;
   }

   //connect to network
     if ( client.isConnected() )
     {
         client.loop();
     }
     else
     {
         client.connect("indoortemp");
     }

}

void publishToParticle(float tempC, float humidityRH)
{
   Particle.publish("tempindoor",
                       String::format("%0.1fC, %0.0f%%",
                           tempC, humidityRH), 60 , PRIVATE);
}

int publishToMQTT(float tempC, float humidityRH)
{
  int publish_successes = 0;

  //client.publish returns 0 if failed, 1 if success
  publish_successes += client.publish("enviro/temperatureIndoor", String(tempC));
  publish_successes += client.publish("enviro/humidityIndoor", String(humidityRH));

  return publish_successes;
}

void publishToSerial(float tempC, float humidityRH)
{
  Serial.print("Temp: "); Serial.println(tempC);
  Serial.print("Humidity: "); Serial.println(humidityRH);
}

//===========================================================
// Temp, Humidity
//===========================================================
//Create Instance of SI7021 temp and humidity sensor
Weather sensor;

float         humidityRHTotal             = 0.0;    //sum of percent %
unsigned int  humidityRHReadingCount      = 0;      //num samples
float         tempCTotal                  = 0.0;    //sum of temps in C
unsigned int  tempCReadingCount           = 0;      //num samples

void initializeTempHumidity()
{
   //Initialize the I2C sensors, and set modes as required
   sensor.begin();
   sensor.enableEventFlags();  //Necessary register calls to enble temp, baro and alt
   return;
}

// Read the humidity sensors, and update the running average
void captureTempHumidity()
{
  // Measure Relative Humidity and temperature from the HTU21D or Si7021
  float humidityRH = sensor.getRH();
  float tempC = sensor.getTemp();

  //Include humidity reading in running mean if within reasonable bounds
  if( humidityRH > 0 && humidityRH < 105 ) // Supersaturation humidity levels over 100% are possible
  {
    humidityRHTotal += humidityRH;
    humidityRHReadingCount++;
  }

  //Include temperature reading in running mean if within reasonable bounds
  if( tempC > -20 && tempC < 80 )
  {
    tempCTotal += tempC;
    tempCReadingCount++;
  }

  return;
}

float getAndResetTempC()
{
   if( tempCReadingCount == 0 )
   {
       return 0;
   }

   float result       = tempCTotal / float( tempCReadingCount );
   tempCTotal         = 0.0;
   tempCReadingCount  = 0;

   return result;
}

float getAndResetHumidityRH()
{
   if( humidityRHReadingCount == 0 )
   {
       return 0;
   }

   float result = humidityRHTotal / float( humidityRHReadingCount );
   humidityRHTotal = 0.0;
   humidityRHReadingCount = 0;

   return result;
}
