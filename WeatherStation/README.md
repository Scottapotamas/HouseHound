# WeatherStation

This node is built with a Particle Photon and a Sparkfun Weather shield.

Uses MQTT to report to the MQTT/Openhab server.

This was a fast turn-key approach to sampling wind, rainfall and wind direction.

Code is somewhat verbose. Bits borrowed from https://github.com/rpurser47/weatherstation with some restructuring.

# Self-heating

The particle photon generates lots of heat when connected to wifi. This causes a self-heating effect which is measured by the temp sensor.

I attempted to reduce the effects by manually controlling the photon's wifi and putting it to sleep between report periods.
Turns out the ubiquiti wifi access point doesn't like a device re-connecting every 5 seconds for days, so this ended up not working.

The sensor's data is really just used for the external sensors on the roof, and barometric pressure.

# TODO

There is a lot of cross-coupled noise between the wind speed and rain reed-switch lines. The actual sensors are on a very long cable.

Need to put a scope on the lines, determine suitable software debounce parameters or beef up the sparkfun board circuitry. Would have been nice for Sparkfun to drive the switches at VCC level, then translate back to 3v3 for the micro.
