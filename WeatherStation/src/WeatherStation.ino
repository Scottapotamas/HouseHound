/*
* Project WeatherStation
* Description:  Uses sparkfun photon weather board with sparkfun weather station.
*               Which includes anemometer, wind vane, and rain meter.
* Author: Scott Rapson,
          sensor interface code from Sparkfun,
          wind/rain interface code adapted from https://github.com/rpurser47/weatherstation
* Date:   March 2017
*/

#include "SparkFun_Photon_Weather_Shield_Library/SparkFun_Photon_Weather_Shield_Library.h"
#include <math.h>

#define SENSOR_SAMPLE_TIME_MS 100
#define PUBLISH_RATE_S 30
#define PUBLISH_RATE_MS (PUBLISH_RATE_S*1000) //seconds into msec

// Timers to maintain sampling and publish throttles
unsigned int timeNextSensorReading;
unsigned int timeNextPublish;

void setup()
{
  //setup the sensing and calculation functions
   initializeTempHumidityAndPressure();
   initializeRainGauge();
   initializeAnemometer();
   initializeWindVane();

   // Schedule the next sensor reading and publish events
   timeNextSensorReading  = millis() + SENSOR_SAMPLE_TIME_MS;
   timeNextPublish        = millis() + PUBLISH_RATE_MS;
}

void loop()
{
   // Sample sensors that need to be polled (temp, humidity, pressure, wind vane)
   // The rain and wind speed sensors use interrupts to catch reed switch closures
   if( timeNextSensorReading <= millis() )
   {
       captureTempHumidityPressure();
       captureWindVane();

       // Schedule the next sensor reading
       timeNextSensorReading = millis() + SENSOR_SAMPLE_TIME_MS;
   }

   // Publish the data collected to Particle and to ThingSpeak
   if( timeNextPublish <= millis() )
   {
       // Get the data to be published
       float tempC        = getAndResetTempC();
       float humidityRH   = getAndResetHumidityRH();
       float pressureKPa  = getAndResetPressurePascals() / 1000.0;

       float rainMillimeters = getAndResetRainMillimeters();
       float gustKMH;
       float windKMH      = getAndResetAnemometerKMH( &gustKMH );
       float windDegrees  = getAndResetWindVaneDegrees();

       // Publish the data
       publishToParticle( tempC, humidityRH, pressureKPa, rainMillimeters, windKMH, gustKMH, windDegrees);

       // Schedule the next publish event
       timeNextPublish = millis() + PUBLISH_RATE_MS;
   }

   delay(10);
}

void publishToParticle(float tempC, float humidityRH, float pressureKPa, float rainMillimeters, float windKMH, float gustKMH, float windDegrees)
{
   Particle.publish("weather",
                       String::format("%0.1fC, %0.0f%%, %0.2fkPa, %0.2fmm, Avg:%0.0fkmh, Gust:%0.0fkmh, Dir:%0.0f deg.",
                           tempC, humidityRH, pressureKPa, rainMillimeters, windKMH, gustKMH, windDegrees), 60 , PRIVATE);
}

//===========================================================
// Temp, Humidity and Pressure
//===========================================================
// The temperature, humidity, and pressure sensors are on board
// the weather station board, and use I2C to communicate.  The sensors are read
// frequently by the main loop, and the results are averaged over the publish cycle

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor library by Sparkfun
Weather sensor;

float         humidityRHTotal             = 0.0;    //sum of percent %
unsigned int  humidityRHReadingCount      = 0;      //num samples
float         tempCTotal                  = 0.0;    //sum of temps in C
unsigned int  tempCReadingCount           = 0;      //num samples
float         pressurePascalsTotal        = 0.0;    //sum of pressure in P
unsigned int  pressurePascalsReadingCount = 0;      //num samples

void initializeTempHumidityAndPressure()
{
   //Initialize the I2C sensors, and set modes as required
   sensor.begin();
   sensor.setModeBarometer();
   sensor.setOversampleRate(7);
   sensor.enableEventFlags();  //Necessary register calls to enble temp, baro and alt
   return;
}

// Read the humidity and pressure sensors, and update the running average
void captureTempHumidityPressure()
{
  // Measure Relative Humidity and temperature from the HTU21D or Si7021
  float humidityRH = sensor.getRH();
  float tempC = sensor.getTemp();

  //Measure Pressure from the MPL3115A2
  float pressurePascals = sensor.readPressure();

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

  //Include pressure reading in running mean if within reasonable bounds
  if( pressurePascals > 80000 && pressurePascals < 110000 )
  {
    pressurePascalsTotal += pressurePascals;
    pressurePascalsReadingCount++;
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

float getAndResetPressurePascals()
{
   if( pressurePascalsReadingCount == 0 )
   {
       return 0;
   }

   float result = pressurePascalsTotal / float( pressurePascalsReadingCount );
   pressurePascalsTotal = 0.0;
   pressurePascalsReadingCount = 0;

   return result;
}

//===========================================================================
// Rain Gauge
//===========================================================================
int RainPin = D2;

volatile unsigned int rainEventCount;
unsigned int          lastRainEvent;
float                 RainScaleInches = 0.011; // Datasheet: Pulse = .011 inches of rain
float                 RainScaleMillimeters = 0.2794; // From imperial value

void initializeRainGauge()
{
 pinMode( RainPin, INPUT_PULLUP );

 rainEventCount = 0;
 lastRainEvent = 0;
 attachInterrupt( RainPin, handleRainEvent, FALLING );

 return;
 }

void handleRainEvent() {
   // Count rain gauge bucket tips as they occur
   unsigned int timeRainEvent = millis(); // grab current time

   // debounce <10mS readings
   if( timeRainEvent - lastRainEvent < 10 ) {
     return;
   }

   rainEventCount++; //Increase this minute's amount of rain
   lastRainEvent = timeRainEvent; // set up for next event
}

float getAndResetRainMillimeters()
{
   float result = RainScaleMillimeters * float( rainEventCount );
   rainEventCount = 0;

   return result;
}

//===========================================================================
// Wind Speed (Anemometer)
//===========================================================================

// The Anemometer generates a frequency relative to the windspeed.  1Hz: 1.492MPH, 2Hz: 2.984MPH, etc.
// We measure the average period (elaspsed time between pulses), and calculate the average windspeed since the last recording.
int AnemometerPin = D3;
float AnemometerScaleMPH = 1.492; // Datasheet: Imperial Windspeed for 1Hz pulses
float AnemometerScaleKMH = 2.4011; // Metric equiv of above

volatile unsigned int AnemoneterPeriodTotal         = 0;
volatile unsigned int AnemoneterPeriodReadingCount  = 0;
volatile unsigned int GustPeriod                    = UINT_MAX;
unsigned int          lastAnemoneterEvent           = 0;

void initializeAnemometer()
{
 pinMode( AnemometerPin, INPUT_PULLUP );

 AnemoneterPeriodTotal = 0;
 AnemoneterPeriodReadingCount = 0;
 GustPeriod = UINT_MAX;  //  The shortest period (and therefore fastest gust) observed
 lastAnemoneterEvent = 0;

 attachInterrupt( AnemometerPin, handleAnemometerEvent, FALLING );

 return;
}

void handleAnemometerEvent()
{
   // Activated by the magnet in the anemometer (2 ticks per rotation)
    unsigned int timeAnemometerEvent = millis(); // grab current time

   //If there's never been an event before (first time), capture it
   if( lastAnemoneterEvent != 0 )
   {
       //time since last event
       unsigned int period = timeAnemometerEvent - lastAnemoneterEvent;

       // debounce <10mS readings (max windspeed of 240km/h)
       if( period < 10 )
       {
         return;
       }

       if(period < GustPeriod)
       {
           // If the period is the shortest (and therefore fastest windspeed) seen, capture it
           GustPeriod = period;
       }

       AnemoneterPeriodTotal += period;
       AnemoneterPeriodReadingCount++;
   }

   lastAnemoneterEvent = timeAnemometerEvent; // set up for next event
}

float getAndResetAnemometerKMH(float * gustKMH)
{
   if( AnemoneterPeriodReadingCount == 0 )
   {
       *gustKMH = 0.0;
       return 0;
   }

   // Have the sum of observed periods between pulses, and number of observations.
   // Calculate the average period (sum / number of readings), take the inverse and multiply by 1000 for frequency, then mulitply by scale factor.
   // The math below is transformed to maximize accuracy by doing all muliplications BEFORE dividing.
   float result = AnemometerScaleKMH * 1000.0 * float( AnemoneterPeriodReadingCount ) / float( AnemoneterPeriodTotal );
   AnemoneterPeriodTotal = 0;
   AnemoneterPeriodReadingCount = 0;
   *gustKMH = AnemometerScaleKMH  * 1000.0 / float( GustPeriod );
   GustPeriod = UINT_MAX;

   return result;
}

//===========================================================
// Wind Vane
//===========================================================
void initializeWindVane()
{
   return;
}

// For the wind vane, average the unit vector components (the sine and cosine of the angle)
int           WindVanePin = A0;

float         windVaneCosTotal      = 0.0;
float         windVaneSinTotal      = 0.0;
unsigned int  windVaneReadingCount  = 0;

void captureWindVane()
{
   //Read the wind vane, and update the running average of the two components of the vector
   unsigned int windVaneRaw = analogRead( WindVanePin );

   float windVaneRadians = lookupRadiansFromRaw( windVaneRaw );

   if( windVaneRadians > 0 && windVaneRadians < 6.14159 )
   {
       windVaneCosTotal += cos( windVaneRadians );
       windVaneSinTotal += sin( windVaneRadians );
       windVaneReadingCount++;
   }

   return;
}

float getAndResetWindVaneDegrees()
{
   if(windVaneReadingCount == 0)
   {
       return 0;
   }

   float avgCos = windVaneCosTotal / float( windVaneReadingCount );
   float avgSin = windVaneSinTotal / float( windVaneReadingCount );
   float result = atan( avgSin / avgCos ) * 180.0 / 3.14159;

   windVaneCosTotal = 0.0;
   windVaneSinTotal = 0.0;
   windVaneReadingCount = 0;

   // atan can only tell where the angle is within 180 degrees.
   // Need to look at cos to tell which half of circle we're in
   if( avgCos < 0 )
   {
     result += 180.0;
   }

   // atan will return negative angles in the NW quadrant
   //push those into positive space.
   if( result < 0 )
   {
     result += 360.0;
   }

  return result;
}

float lookupRadiansFromRaw(unsigned int analogRaw)
{
   // The mechanism for reading the weathervane isn't arbitrary, but just look up which of 16 position we're in.
   if( analogRaw >= 2200 && analogRaw < 2400 ) return ( 3.14 );   //South
   if( analogRaw >= 2100 && analogRaw < 2200 ) return ( 3.53 );   //SSW
   if( analogRaw >= 3200 && analogRaw < 3299 ) return ( 3.93 );   //SW
   if( analogRaw >= 3100 && analogRaw < 3200 ) return ( 4.32 );   //WSW
   if( analogRaw >= 3890 && analogRaw < 3999 ) return ( 4.71 );   //West
   if( analogRaw >= 3700 && analogRaw < 3780 ) return ( 5.11 );   //WNW
   if( analogRaw >= 3780 && analogRaw < 3890 ) return ( 5.50 );   //NW
   if( analogRaw >= 3400 && analogRaw < 3500 ) return ( 5.89 );   //NNW
   if( analogRaw >= 3570 && analogRaw < 3700 ) return ( 0.00 );   //North
   if( analogRaw >= 2600 && analogRaw < 2700 ) return ( 0.39 );   //NNE
   if( analogRaw >= 2750 && analogRaw < 2850 ) return ( 0.79 );   //NE
   if( analogRaw >= 1510 && analogRaw < 1580 ) return ( 1.18 );   //ENE
   if( analogRaw >= 1580 && analogRaw < 1650 ) return ( 1.57 );   //East
   if( analogRaw >= 1470 && analogRaw < 1510 ) return ( 1.96 );   //ESE
   if( analogRaw >= 1900 && analogRaw < 2000 ) return ( 2.36 );   //SE
   if( analogRaw >= 1700 && analogRaw < 1750 ) return ( 2.74 );   //SSE

   if(analogRaw > 4000) return(-1);     // Open circuit?  Probably means the sensor is not connected

   Particle.publish("error", String::format("Got %d from Windvane.", analogRaw), 60 , PRIVATE);
   return -1;
}
