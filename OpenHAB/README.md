# OpenHAB Notes

Setup a OpenHAB 2 install on a Pi2.

# General notes for myself

- Changed the timezone and local to Adelaide etc.
- Changed the default password and samba password, still using openhabian as the user
- Configured mosquitto as the MQTT broker
- Tested the mqtt server using MQTT.fx on windows

- Assigned the pi to 192.168.1.100 via DHCP reservation.
- Mount the samba share and edit config files as required. WebUI seems painful to use.

- Hostname is househound now instead of openhabian.
- Port 8080 has the webUI for OpenHAB.

- In the mqtt.cfg config file, 192.168.1.100:8083 was set as the broker URI for the mosquitto install done earlier.

# Persistance Setup

Using mySQL for persistence of data. Database is Openhab, user is openhab with standard password.

- Temp/Humidity and Pressure are logged at 30min interval.
- Rainfall is logged on every update as each update represents non-zero rainfall over the sampling period.
- A series of persistance periods are added. Need to work out what to use for each dataset.