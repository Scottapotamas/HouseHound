# HouseHound

Home datalogging and display system for DIY (TCP/IP based) sensor nodes and web facing display.

I use a Raspberry Pi running OpenHAB 2 as the main hub, and then a bunch of stateless microcontroller nodes around the house interface over wifi and ethernet.

This repo contains all the applicable files for hardware, fimware, or notes for setup for each of the projects to consolidate everything, but any combination of parts should work alone. This repo is more for self-documenting the project than anything else, as keeping track of configuration is tedious.

# WeatherStation

![Weather](Images/WeatherPanel.png?raw=true "WeatherStation View")

Large and high feature set weather monitoring to measure the following info:
- Temperature
- Humidity
- Barometric Pressure
- Wind Speed
- Wind Direction
- Rainfall

Reference data is also scraped from the Australian BoM page for our closest station for temp/humidity references. From watching the BoM reference against local sensors, my system tracks the 'reference' data within 1°C and ~5°% RH over most days, well within the realm of local weather differences due to ~3km distance from that station.

## TempHumidityNodes

Several of these are sitting around the house to provide 2-zone indoor monitoring, and outdoor sensing.

Small and low cost Wifi Weather node for data collection, also sent to a local OpenHAB instance.

- Uses a particle photon wifi based microcontroller.
- Temperature and Humidity Sensor, includes additional spare IO for future expansion.
- Uses MQTT to send averaged temperature and humidity data to the mosquitto server.

## WeatherStation

This sits on a pole mounted to the roof. Leverages a Particle Photon wireless microcontroller and the Sparkfun weather shield to simplify connections to the hardware. Uses interrupts for rain/wind speed events, and polls other sensors periodically.

Sensors are sampled at 100ms, and averaged across the time between messages to the openHAB server for persistance and processing.

We send temperature, humidity, rainfall accumulated, windspeed and wind direction every few seconds, and track long term averages of wind speed and baro for lower noise. The short term values are shown for more responsive dashboards, while the internally averaged min/max values are logged for graphing and stats use.

MQTT is once again used to send the values to the mosquitto broker, which the OpenHAB instance subscribes to.

# PowerMonitor

![Power](Images/PowerPanel.png?raw=true "PowerStation View")

A bunch of current transducers and 3 12VAC transformers in the sub-distro board. This allows for monitoring of the voltage and current of each phase (have 3-phase installed).

Sends data to local MQTT broker as done with other sensors.

Measures:
- 3-phase current 
- 3-phase Voltage
- Power factor
- Frequency

# Airconditioner Controller

IN PROGRESS

Replacement control module for the evaporative aircon unit. Provides better information display, finer grained control of hardware, intelligent modes.

I pull data from a local OpenHAB instance for another weather node in the house, and scrapped internet weather info.
HAB node also provides some level of control for home automation.

Uses following hardware:
- Adafruit HUZZAH ESP-12E module
- HDC1000 temp and humidity sensor
- BMP280 pressure sensor
- SSD1306 2.7inch OLED (on i2c)
- WS2812 (RGBW) as a status lamp

# WaterMonitor

Sensors to monitor the status of the rain water tanks and pump system.
With a known tank level and flow information over a daily/weekly time period it should be possible to predict time-to-empty.

Measures:
- Water level in the tank using a submerged pressure sensor.
- Water flow from tank (TODO).

## Tank Level

Uses a $60 Aliexpress sourced 'stainless' 4-20mA pressure sensor, a particle photon and a ADS1115 ADC breakout from adafruit as these were just spares I had on hand. The particle and ADC are powered with a small pololu 500mA 5V SMPS which was spare.

Threw a couple of 100uF electro's on the supplies for stability, and put it in a waterproof box for outdoor use with an old phone charger to supply 12ish volts.

A gland was used to draw the pressure sensor's cable from the tank.

Because the sensor is a 4-20mA loop device, I use the ADC in differential mode across a 220R resistor wit


Eventually I'd like to add the ability to switch the feed valve from the rain collection sump to allow user controllable bypass of the tanks, so the first few mm of rain are purged and we don't put the accumulated dirt from the roof into the tanks. After manual implementation, may consider doing this automatically or semi-autonomously based on rain data.

A AC powered rotary valve with DN40 fittings was purchased from Aliexpress to test with this.

# Other OpenHAB integration

The openhab server uses HabPanel for most visualisation, which is viewed on a older Microsoft surface in fullscreen mode.

The server performs hourly speedtest.net tests and logs/graphs down/up/ping. SMNP integration to read wifi/nas utilisation is also planned.

Todo:

- Integrate with Chromecasts for control over music/tv outputs.
- Connect the burglar alarm RS232 to gain access to PIR and state information?
- Add solenoid control to the side gate
