#include "Adafruit_ADS1X15.h"
#include "MQTT.h"

#define LONG_TERM_AVERAGE_M 1
#define LONG_TERM_BUFFER_SIZE ((LONG_TERM_AVERAGE_M*60)/PUBLISH_RATE_S) //sample pubs (sec) into longterm period (min to seconds)

#define SENSOR_SAMPLE_TIME_MS 250
#define PUBLISH_RATE_S 5
#define PUBLISH_RATE_MS (PUBLISH_RATE_S*1000) //seconds into msec

// Timers to maintain sampling and publish throttles
unsigned int timeNextSensorReading;
unsigned int timeNextPublish;

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

void callback(char* topic, byte* payload, unsigned int length);


byte server[] = { 192,168,1,100 };
MQTT client(server, 1883, callback);

// recieve message
void callback(char* topic, byte* payload, unsigned int length) 
{
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  delay(1000);
}

void setup(void) 
{
  client.connect("watermonitor");

// publish/subscribe
  if (client.isConnected())
  {
    client.publish("debug/watermonitor","Starting Up");
  }

// The ADC input range (or gain) can be changed via the following
// functions, but be careful NEVER TO EXCEED +0.3V OVER VDD ON GAINED INPUTS,
// or exceed the upper and lower limits if you adjust the input range!
// Setting these values incorrectly may DESTROY your ADC!
//
//  *** TAKE CARE WHEN SETTING GAIN TO NOT EXCEED ABOVE LIMITS ON INPUT!!
//                                                                    ADS1015   ADS1115
//                                                                    -------   -------
ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit =       3mV       0.1875mV (DEFAULT)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit =     2mV       0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit =     1mV       0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit =     0.5mV     0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit =     0.25mV    0.015625mV
// ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit =     0.125mV   0.0078125mV  
ads.begin();

timeNextSensorReading  = millis() + SENSOR_SAMPLE_TIME_MS;
timeNextPublish        = millis() + PUBLISH_RATE_MS;
}

void loop(void)
{
  if( timeNextSensorReading <= millis() )
  {
    sampleLevel();

    // Schedule the next sensor reading
    timeNextSensorReading = millis() + SENSOR_SAMPLE_TIME_MS;
  }

  // Publish the data collected to MQTT
  if( timeNextPublish <= millis() )
  {
    // Get the data to be published
    float tankHeight = getAndResetHeight();
    client.publish("enviro/tankDepthMM", String(tankHeight) );

    timeNextPublish = millis() + PUBLISH_RATE_MS;  // Schedule the next publish event
  }

  //connect to network
  if( client.isConnected() )
  {
    client.loop();
  }
  else
  {
    client.connect("weatherstation");
  }
}

float         pressADCTotal     = 0.0;    //sum of adc counts
unsigned int  pressADCReadingCount = 0;      //num samples

void sampleLevel()
{
  double multiplier = 0.1875F; //milli Volts per bit for ADS1115

  short adc0_1 = ads.readADC_Differential_0_1();  
  double av0_1 = adc0_1 * multiplier;

  pressADCTotal += av0_1;
  pressADCReadingCount++;
}

float getAndResetHeight()
{
  if( pressADCReadingCount == 0 )
  {
    return 0;
  }

  float result       = pressADCTotal / float( pressADCReadingCount );
  result = convert_voltage_to_depth( result );  //convert the averaged ADC readings into height in mm

  pressADCTotal         = 0.0;
  pressADCReadingCount  = 0;

  return result;
}

float convert_voltage_to_depth(float milliVolts)
{
  float depth_in_mm = 0;

  //Curve from measured voltage vs depth information is
  // y = 1.6976x - 1465.5 where x is in mV
  depth_in_mm = (1.6976 * milliVolts) - 1465.5;


  if (depth_in_mm < 0)
  {
    depth_in_mm = 0;
  }
  
  return depth_in_mm;
}
