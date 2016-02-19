# HouseHound
Home datalogging and display system for DIY (IP based) sensor nodes and web facing display.



# Airconditioner Controller
Replacement control module for the evaporative aircon unit. Provides better information display, finer grained control of hardware, intelligent modes.
I pull data from a local OpenHAB instance for another weather node in the house, and scrapped internet weather info.
HAB node also provides some level of control for home automation.

Uses following hardware:
- Adafruit HUZZAH ESP-12E module
- HDC1000 temp and humidity sensor
- BMP280 pressure sensor
- SSD1306 2.7inch OLED (on i2c)
- WS2812 (RGBW) as a status lamp


# WeatherNode

Small and low cost Wifi Weather node for outdoor data, sent to a local OpenHAB instance.

Uses Adafruit HUZZAH ESP-12E
Temperature and Humidity Sensor.

# WeatherNodeExpanded
Large and high feature set weather node to measure the following info:
- Temperature
- Humidity
- Wind Speed
- Wind Direction
- Rainfall
- Light levels

Sends data back to the local openHAB server.

# MeterMonitor
A bunch of current transducers and 12VAC transformers in the sub-distro board. 
Uses wifi chip to isolate from networking stack. Sends data to local openHAB server.

Measures:
- 3-phase current 
- 3-phase Voltage
- Power factor
- Frequency


# WaterMonitor
Sensors to monitor the status of the rain water tanks and pump system.
With a known tank level and flow information over a daily/weekly time period it should be possible to predict time-to-empty.

Measures 
- Water level in the tank
- Water flow from tank
