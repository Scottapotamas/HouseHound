# Temp and Humidity Node

This node is built with a Particle Photon and a Si7021 i2c based temperature and humidity IC.

Uses MQTT to report to the MQTT/Openhab server.

These boards are design to be as simple and fast to deploy as possible. Idea is to drop one at each end of the house, and one or two outside.

# Self-heating (or lack of)

The particle photon generates lots of heat when connected to wifi. This causes a self-heating effect on the weathershield which uses the same sensor.

As these are wired directly, the sensor is on about 100mm of free cable which will allow flexibility in isolating the sensor from the photon.