# HouseHound
Home datalogging and display system for DIY (TCP/IP based) sensor nodes and web facing display.

I use a Raspberry Pi running OpenHAB as the main hub, and then a bunch of stateless microcontroller nodes around the house interface over wifi and ethernet.

This repo contains all the applicable files for hardware, fimware, or notes for setup for each of the projects to consolidate everything, but any combination of parts should work alone. This repo is more for self-documenting the project than anything else, as keeping track of configuration is tedious.

# Airconditioner Controller

Replacement control module for the evaporative aircon unit. 

Provides better information display, finer grained control of hardware, intelligent modes.

I pull data from a local OpenHAB instance for another weather node in the house, and scrapped internet weather info.
HAB node also provides some level of control for home automation.

Uses following hardware:
- Adafruit HUZZAH ESP-12E module
- HDC1000 temp and humidity sensor
- BMP280 pressure sensor
- SSD1306 2.7inch OLED (on i2c)
- WS2812 (RGBW) as a status lamp

# WeatherNode

Small and low cost Wifi Weather node for outdoor data collection, also sent to a local OpenHAB instance.

Uses Adafruit HUZZAH ESP-12E
Temperature and Humidity Sensor, includes additional spare IO for future expansion.
Has hardware provision to run on a lithium battery such as an 18650 with solar panel.
Uses MQTT to send data to the mosquitto server.

# WeatherStation

Large and high feature set weather node to measure the following info:
- Temperature
- Humidity
- Barometric Pressure
- Wind Speed
- Wind Direction
- Rainfall

This sits on a pole mounted to the roof. Leverages a Particle Photon wireless microcontroller and the Sparkfun weather shield to simplify connections to the hardware. Uses interrupts for rain/wind speed events, and polls other sensors periodically.

Sensors are sampled at 100ms, and averaged across the time between messages to the openHAB server for persistance and processing.

We send temperature, humidity, rainfall accumulated and barometric pressure every minute, while windspeed, gusts and wind direction are sent far faster for a more interactive viewing experience.

MQTT is used to send the values to the mosquitto broker.

# MeterMonitor

A bunch of current transducers and 3 12VAC transformers in the sub-distro board. This allows for monitoring of the voltage and current of each phase (have 3-phase installed).

Uses wifi chip to electrically isolate from networking stack. Sends data to local openHAB server.

Measures:
- 3-phase current 
- 3-phase Voltage
- Power factor
- Frequency

Will run off a 18650 which allows for monitoring and logging when the supply is cut.

# WaterMonitor

Sensors to monitor the status of the rain water tanks and pump system.
With a known tank level and flow information over a daily/weekly time period it should be possible to predict time-to-empty.

Measures:
- Water level in the tank using a weatherproof ultrasonic sensor.
- Water flow from tank (TODO, find a suitable flow meter that isn't plastic).

Eventually I'd like to add the ability to switch the feed valve from the rain collection sump to allow user controllable bypass of the tanks, so the first few mm of rain are purged and we don't put the accumulated dirt from the roof into the tanks. After manual implementation, may consider doing this automatically or semi-autonomously based on rain data.
