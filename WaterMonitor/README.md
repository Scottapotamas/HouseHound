# WaterMonitor

Uses particle photon, ADS1115 ADC and 220R resistor as burden for the 4-20mA pressure sensor from Aliexpress.

Samples every 250ms, averages samples over 5 seconds and publishes to the MQTT server every 5 seconds.

This is overkill and over-accurate in every aspect.