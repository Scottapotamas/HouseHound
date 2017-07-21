/* ATM90E36 Energy Monitor

  Uses a ATM90E36 breakout board by whatnick fron Tindie.
  Uses MQTT to talk to OpenHab system for power logging/graphing.
  Borrowed bits of their (whatnick and Ryzee)'s library for interfacing, and adapted to Photon.

  Scott Rapson - 2017
*/

#define SoftReset 0x00    //Software Reset
#define SysStatus0 0x01   //System Status0
#define SysStatus1 0x02   //System Status1
#define FuncEn0 0x03      //Function Enable0
#define FuncEn1 0x04      //Function Enable1
#define SagTh 0x08        //Voltage Sag Threshold
//#define SmallPMod 0x04  //Small-Power Mode
#define LastSPIData 0x0F  //Last Read/Write SPI Value
//#define LSB 0x08        //RMS/Power 16-bit LSB

#define ConfigStart 0x30  //Configuration start command
#define PLconstH 0x31     //High Word of PL_Constant
#define PLconstL 0x32     //Low Word of PL_Constant
#define MMode0 0x33       //Metering Mode Configuration
#define MMode1 0x34       //Metering Mode Configuration
#define PStartTh 0x35     //Active Startup Power Threshold
#define QStartTh 0x36     //Reactive Startup Power Threshold
#define CSZero 0x3B	      //Checksum 0

#define CalStart 0x40     //Calibration Start Command
#define PoffsetA 0x41     //L Line Active Power Offset
#define QoffsetA 0x42     //L Line Reactive Power Offset
#define PoffsetB 0x43     //L Line Active Power Offset
#define QoffsetB 0x44     //L Line Reactive Power Offset
#define PoffsetC 0x45     //L Line Active Power Offset
#define QoffsetC 0x46     //L Line Reactive Power Offset
#define GainA 0x47        //A Line Calibration Gain
#define PhiA 0x48         //A Line Calibration Angle
#define GainB 0x49        //B Line Calibration Gain
#define PhiB 0x4A         //B Line Calibration Angle
#define GainC 0x4B        //C Line Calibration Gain
#define PhiC 0x4C         //C Line Calibration Angle
#define CSOne 0x4D        //Checksum 1

#define AdjStart 0x60     //Measurement Calibration Start Command
#define UgainA 0x61       //A Voltage rms Gain
#define IgainA 0x62       //A Line Current rms Gain
#define UgainB 0x65       //B Voltage rms Gain
#define UgainC 0x69       //C Voltage rms Gain
#define IgainB 0x66       //B Line Current rms Gain
#define IgainC 0x6A       //C Line Current rms Gain
#define UoffsetA 0x63     //Voltage Offset
#define UoffsetB 0x67     //Voltage Offset
#define UoffsetC 0x6B     //Voltage Offset
#define IoffsetA 0x64     //L Line Current Offset
#define IoffsetB 0x68     //N Line Current Offse
#define IoffsetC 0x6C     //N Line Current Offse
#define CSThree 0x6F      //Checksum 3

#define APenergyA 0x81    //Forward Active Energy
#define APenergyB 0x82    //Forward Active Energy
#define APenergyC 0x83    //Forward Active Energy
#define ANenergyA 0x85    //Reverse Active Energy
#define ANenergyB 0x86    //Reverse Active Energy
#define ANenergyC 0x87    //Reverse Active Energy
#define ANenergyT 0x84    //Absolute Active Energy
#define RPenergyA 0x89    //Forward (Inductive) Reactive Energy
#define RPenergyB 0x8A    //Forward (Inductive) Reactive Energy
#define RPenergyC 0x8B    //Forward (Inductive) Reactive Energy
#define RnenergyA 0x8D    //Reverse (Capacitive) Reactive Energy
#define RnenergyB 0x8E    //Reverse (Capacitive) Reactive Energy
#define RnenergyC 0x8F    //Reverse (Capacitive) Reactive Energy
#define RPenergyT 0x88    //Absolute Reactive Energy
#define EnStatus0 0x95    //Metering Status
#define EnStatus1 0x96    //Metering Status
#define IrmsA 0xDD        //L Line Current rms A
#define IrmsB 0xDE        //L Line Current rms B
#define IrmsC 0xDF        //L Line Current rms C
#define UrmsA 0xD9        //Voltage rms A
#define UrmsB 0xDA        //Voltage rms B
#define UrmsC 0xDB        //Voltage rms C

#define PmeanT 0xB0     //Total Mean Active Power
#define PmeanA 0xB1     //A Line Mean Active Power
#define PmeanB 0xB2     //B Line Mean Active Power
#define PmeanC 0xB3     //C Line Mean Active Power
#define QmeanA 0xB5     //A Line Mean Reactive Power
#define QmeanB 0xB6     //B Line Mean Reactive Power
#define QmeanC 0xB7     //C Line Mean Reactive Power
#define SmeanA 0xB9     //A Line Mean Apparent Power
#define SmeanB 0xBA     //B Line Mean Apparent Power
#define SmeanC 0xBB     //C Line Mean Apparent Power
#define PFmeanT 0xBC    //Mean Power Factor

#define Freq 0xF8       //Voltage Frequency
#define PangleA 0xF9    //Phase Angle between Voltage and A Line Current
#define PangleB 0xFA    //Phase Angle between Voltage and B Line Current
#define PangleC 0xFB    //Phase Angle between Voltage and C Line Current

#include "MQTT.h"
#define SENSOR_SAMPLE_TIME_MS 1000
#define PUBLISH_RATE_S 5
#define PUBLISH_RATE_MS (PUBLISH_RATE_S*1000) //seconds into msec

// Timers to maintain sampling and publish throttles
unsigned int timeNextSensorReading;
unsigned int timeNextPublish;

void callback(char* topic, byte* payload, unsigned int length);

/**
 * if want to use IP address,
 * byte server[] = { XXX,XXX,XXX,XXX };
 * MQTT client(server, 1883, callback);
 **/
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

void setup()
{
  /* Initialize the serial port to host */
  Serial.begin(115200);

  client.connect("PowerMonitor");
  // publish/subscribe
  if (client.isConnected())
  {
      client.publish("debug/weatherstation","Starting Up");
  }

  /*Initialise the ATM90E36 + SPI port */
  InitEnergyIC();

  timeNextSensorReading  = millis() + SENSOR_SAMPLE_TIME_MS;
  timeNextPublish        = millis() + PUBLISH_RATE_MS;
}

void loop()
{
  if( timeNextSensorReading <= millis() )
  {
      captureVoltages();
      captureCurrents();
      captureOther();

      // Schedule the next sensor reading
      timeNextSensorReading = millis() + SENSOR_SAMPLE_TIME_MS;
  }

  // Publish the data collected to MQTT
  if( timeNextPublish <= millis() )
  {
     // Publish the data
     publishToMQTT(get_averaged_voltageA(),
                    get_averaged_voltageB(),
                    get_averaged_voltageC(),
                    get_averaged_currentA(),
                    get_averaged_currentB(),
                    get_averaged_currentC(),
                    get_averaged_powerfactor(),
                    get_averaged_frequency()
                    );
/*
    publishToSerial(get_averaged_voltageA(),
                   get_averaged_voltageB(),
                   get_averaged_voltageC(),
                   get_averaged_currentA(),
                   get_averaged_currentB(),
                   get_averaged_currentC(),
                   get_averaged_powerfactor(),
                   get_averaged_frequency()
                   );
*/
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
      client.connect("powermonitor");
  }
}

//===========================================================
// Getting data out of the photon...
//===========================================================

void publishToMQTT(float v_A, float v_B, float v_C, float i_A, float i_B, float i_C, float pf, float freq)
{
  client.publish( "power/voltsA", String(v_A) );
  client.publish( "power/voltsB", String(v_B) );
  client.publish( "power/voltsC", String(v_C) );

  client.publish( "power/currentA", String(i_A) );
  client.publish( "power/currentB", String(i_B) );
  client.publish( "power/currentC", String(i_C) );

  //TODO publish real power (V*I*cosTheta) for each Phase
  //TODO publish sum of powers for "house power?"

  client.publish( "power/powerfactor", String(pf) );
  client.publish( "power/frequency", String(freq) );

}

void publishToSerial(float v_A, float v_B, float v_C, float i_A, float i_B, float i_C, float pf, float freq)
{
  Serial.println("Phase A: " + String(v_A) + "V " + String(i_A) + "A");
  Serial.println("Phase B: " + String(v_B)+ "V " + String(i_B) + "A");
  Serial.println("Phase C: " + String(v_C) + "V " + String(i_C) + "A");
  Serial.println("Power Factor: " + String(pf));
  Serial.println("Frequency: " + String(freq) + "Hz");
  Serial.println("");
}

//===========================================================
// Status Registers
//===========================================================

void checkStatus()
{
  int sys0 = GetSysStatus0();
  int sys1 = GetSysStatus1();
  int en0 = GetMeterStatus0();
  int en1 = GetMeterStatus1();

  Serial.println("S0: 0x" + String(sys0,HEX) );
  Serial.println("S1: 0x" + String(sys1,HEX) );
  Serial.println("E0: 0x" + String(en0,HEX) );
  Serial.println("E1: 0x" + String(en1,HEX) );
}

//===========================================================
// Line Voltages
//===========================================================

double voltageA_total,voltageB_total,voltageC_total;
int voltageA_count,voltageB_count,voltageC_count;

void captureVoltages()
{
  float voltageA, voltageB, voltageC;

  voltageA = GetLineVoltageA();
  voltageB = GetLineVoltageB();
  voltageC = GetLineVoltageC();

  if( voltageA > 0 && voltageA < 100 )
  {
    voltageA_total += voltageA;
    voltageA_count++;
  }

  if( voltageB > 0 && voltageB < 100 )
  {
    voltageB_total += voltageB;
    voltageB_count++;
  }

  if( voltageC > 0 && voltageC < 100 )
  {
    voltageC_total += voltageC;
    voltageC_count++;
  }
}

float get_averaged_voltageA()
{
  return voltageA_total / voltageA_count;
}

float get_averaged_voltageB()
{
  return voltageB_total / voltageB_count;
}

float get_averaged_voltageC()
{
  return voltageC_total / voltageC_count;
}

//===========================================================
// Current Values
//===========================================================

double currentA_total,currentB_total,currentC_total;
int currentA_count,currentB_count,currentC_count;

void captureCurrents()
{
  float currentA, currentB, currentC;

  currentA = GetLineCurrentA();
  currentB = GetLineCurrentB();
  currentC = GetLineCurrentC();

  if( currentA > 0 && currentA < 100 )
  {
    currentA_total += currentA;
    currentA_count++;
  }

  if( currentB > 0 && currentB < 100 )
  {
    currentB_total += currentB;
    currentB_count++;
  }

  if( currentC > 0 && currentC < 100 )
  {
    currentC_total += currentC;
    currentC_count++;
  }
}

float get_averaged_currentA()
{
  return currentA_total / currentA_count;
}

float get_averaged_currentB()
{
  return currentB_total / currentB_count;
}

float get_averaged_currentC()
{
  return currentC_total / currentC_count;
}

//===========================================================
// Frequency and Power Factor
//===========================================================

double freq_total,pf_total;
int freq_count,pf_count;

void captureOther()
{
  float freq, pf;

  freq = GetFrequency();
  pf = GetPowerFactor();

  //Include frequency reading if within bounds
  if( freq > 0 && freq < 100 )
  {
    freq_total += freq;
    freq_count++;
  }

  //Include power factor reading if within bounds
  if( pf > 0 && pf < 1 )
  {
    pf_total += pf;
    pf_count++;
  }
}

float get_averaged_frequency()
{
  return freq_total / freq_count;
}

float get_averaged_powerfactor()
{
  return pf_total / pf_count;
}

//===========================================================
// ATM90E36 Handling and Interfacing
//===========================================================

void InitEnergyIC()
{
  unsigned short systemstatus0;
  pinMode(A2, OUTPUT );

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  CommEnergyIC(0, SoftReset, 0x789A); //Perform soft reset
  CommEnergyIC(0, FuncEn0, 0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  CommEnergyIC(0, FuncEn1, 0x0030); //Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
  CommEnergyIC(0, SagTh, 0x1F2F); //Voltage sag threshhold

  //Set metering config values
  CommEnergyIC(0, ConfigStart, 0x5678); //Metering calibration startup command. Register 31 to 3B need to be set
  CommEnergyIC(0, PLconstH, 0x00B9); //PL Constant MSB
  CommEnergyIC(0, PLconstL, 0xC1F3); //PL Constant LSB
  CommEnergyIC(0, MMode0, 0x0087); //Metering Mode Configuration. All defaults. See pg 58 of datasheet.
  CommEnergyIC(0, MMode1, 0x5555); //PGA Gain Configuration. x2 for DPGA and PGA. See pg 59 of datasheet
  CommEnergyIC(0, PStartTh, 0x08BD); //Active Startup Power Threshold
  CommEnergyIC(0, QStartTh, 0x0AEC); //Reactive Startup Power Threshold
  CommEnergyIC(0, CSZero, 0x5F59); //Write CSOne, as self calculated

  Serial.print("Checksum 0:");
  Serial.println(CommEnergyIC(1, CSZero, 0x0000), HEX); //Checksum 0. Needs to be calculated based off the above values.

  //Set metering calibration values
  CommEnergyIC(0, CalStart, 0x5678); //Metering calibration startup command. Register 41 to 4D need to be set
  CommEnergyIC(0, GainA, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiA, 0x0000); //Line calibration angle
  CommEnergyIC(0, GainB, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiB, 0x0000); //Line calibration angle
  CommEnergyIC(0, GainC, 0x1D39); //Line calibration gain
  CommEnergyIC(0, PhiC, 0x0000); //Line calibration angle
  CommEnergyIC(0, PoffsetA, 0x0000); //A line active power offset
  CommEnergyIC(0, QoffsetA, 0x0000); //A line reactive power offset
  CommEnergyIC(0, PoffsetB, 0x0000); //B line active power offset
  CommEnergyIC(0, QoffsetB, 0x0000); //B line reactive power offset
  CommEnergyIC(0, PoffsetC, 0x0000); //C line active power offset
  CommEnergyIC(0, QoffsetC, 0x0000); //C line reactive power offset
  CommEnergyIC(0, CSOne, 0x2402); //Write CSOne, as self calculated

  Serial.print("Checksum 1:");
  Serial.println(CommEnergyIC(1, CSOne, 0x0000), HEX); //Checksum 1. Needs to be calculated based off the above values.

  //Set measurement calibration values
  CommEnergyIC(0, AdjStart, 0x5678); //Measurement calibration startup command, registers 61-6F
  CommEnergyIC(0, UgainA, 0xD8E9);  //A SVoltage rms gain
  CommEnergyIC(0, IgainA, 0x1BC9); //A line current gain
  CommEnergyIC(0, UoffsetA, 0x0000); //A Voltage offset
  CommEnergyIC(0, IoffsetA, 0x0000); //A line current offset
  CommEnergyIC(0, UgainB, 0xD8E9);  //B Voltage rms gain
  CommEnergyIC(0, IgainB, 0x1BC9); //B line current gain
  CommEnergyIC(0, UoffsetB, 0x0000); //B Voltage offset
  CommEnergyIC(0, IoffsetB, 0x0000); //B line current offset
  CommEnergyIC(0, UgainC, 0xD8E9);  //C Voltage rms gain
  CommEnergyIC(0, IgainC, 0x1BC9); //C line current gain
  CommEnergyIC(0, UoffsetC, 0x0000); //C Voltage offset
  CommEnergyIC(0, IoffsetC, 0x0000); //C line current offset
  CommEnergyIC(0, CSThree, 0xA694); //Write CSThree, as self calculated

  Serial.print("Checksum 3:");
  Serial.println(CommEnergyIC(1, CSThree, 0x0000), HEX); //Checksum 3. Needs to be calculated based off the above values.

  CommEnergyIC(0, ConfigStart, 0x8765); //Checks correctness of 31-3B registers and starts normal metering if ok
  CommEnergyIC(0, CalStart, 0x8765); //Checks correctness of 41-4D registers and starts normal metering if ok
  CommEnergyIC(0, AdjStart, 0x8765); //Checks correct ness of 61-6F registers and starts normal measurement  if ok

  systemstatus0 = GetSysStatus0();

  if (systemstatus0 & 0x4000)
  {
    //checksum 1 error
    Serial.println("checksum error 0");
  }

  if (systemstatus0 & 0x1000)
  {
    //checksum 2 error
     Serial.println("checksum error 1");
  }

  if (systemstatus0 & 0x0400)
  {
    //checksum 2 error
     Serial.println("checksum error 2");
  }

  if (systemstatus0 & 0x0100)
  {
    //checksum 2 error
     Serial.println("checksum error 3");
  }
}

unsigned short CommEnergyIC(unsigned char RW, unsigned short address, unsigned short val)
{
  unsigned char* data = (unsigned char*)&val;
  unsigned char* adata = (unsigned char*)&address;
  unsigned short output;
  unsigned short address1;

  //SPI interface rate is 200 to 160k bps. It will need to be slowed down for EnergyIC
    SPI.setClockSpeed(200000);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE3);

  //switch MSB and LSB of value
  output = (val >> 8) | (val << 8);
  val = output;

  //Set read write flag
  address |= RW << 15;
  //Swap address bytes
  address1 = (address >> 8) | (address << 8);
  address = address1;


  //Transmit and receive data
  SPI.beginTransaction();

  //enable chip and wait for SPI bus to activate
  digitalWrite (A2, LOW);
  delayMicroseconds(10);

  //Write address byte by byte
  for (byte i=0; i<2; i++)
  {
    SPI.transfer (*adata);
    adata++;
  }

  // Must wait 4 us for data to become valid
  delayMicroseconds(4);

  //Read data for each byte in transfer
  if (RW)
  {
	  for (byte i=0; i<2; i++)
    {
      *data = SPI.transfer (0x00);
      data++;
    }
  }
  else
  {
	for (byte i=0; i<2; i++)
    {
      SPI.transfer(*data);  // write all the bytes
      data++;
    }
  }

  digitalWrite(A2, HIGH);
  delayMicroseconds(10);

  SPI.endTransaction();

  output = (val >> 8) | (val << 8); //reverse MSB and LSB
  return output;
}

double GetLineVoltageA()
{
  unsigned short voltage = CommEnergyIC(1, UrmsA, 0xFFFF);
  return (double)voltage / 238.5;
}

double GetLineVoltageB()
{
  unsigned short voltage = CommEnergyIC(1, UrmsB, 0xFFFF);
  return (double)voltage / 238.5;
}

double GetLineVoltageC()
{
  unsigned short voltage = CommEnergyIC(1, UrmsC, 0xFFFF);
  return (double)voltage / 238.5;
}

unsigned short GetMeterStatus0()
{
  return CommEnergyIC(1, EnStatus0, 0xFFFF);
}

unsigned short GetMeterStatus1()
{
  return CommEnergyIC(1, EnStatus1, 0xFFFF);
}
double GetLineCurrentA()
{
  unsigned short current = CommEnergyIC(1, IrmsA, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double GetLineCurrentB()
{
  unsigned short current = CommEnergyIC(1, IrmsB, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double GetLineCurrentC()
{
  unsigned short current = CommEnergyIC(1, IrmsC, 0xFFFF);
  return (double)current * 7.13 / 1000;
}

double GetActivePowerA()
{
  short int apower = (short int)CommEnergyIC(1, PmeanA, 0xFFFF); //Complement, MSB is signed bit
  return (double)apower * 2.94;
}

double GetFrequency()
{
  unsigned short freq = CommEnergyIC(1, Freq, 0xFFFF);
  return (double)freq / 100;
}

double GetPowerFactor()
{
  short int pf = (short int)CommEnergyIC(1, PFmeanT, 0xFFFF); //MSB is signed bit
  //if negative
  if (pf & 0x8000) {
    pf = (pf & 0x7FFF) * -1;
  }
  return (double)pf / 1000;
}

double GetImportEnergy()
{
  //Register is cleared after reading
  unsigned short ienergy = CommEnergyIC(1, APenergyA, 0xFFFF);
  return (double)ienergy / 10 / 1000; //returns kWh if PL constant set to 1000imp/kWh
}

double GetExportEnergy()
{
  //Register is cleared after reading
  unsigned short eenergy = CommEnergyIC(1, ANenergyT, 0xFFFF);
  return (double)eenergy / 10 / 1000; //returns kWh if PL constant set to 1000imp/kWh
}

unsigned short GetSysStatus0()
{
  return CommEnergyIC(1, SysStatus0, 0xFFFF);
}

unsigned short GetSysStatus1()
{
  return CommEnergyIC(1, SysStatus1, 0xFFFF);
}
