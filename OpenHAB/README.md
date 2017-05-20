# OpenHAB Notes

Setup a OpenHAB 2 install on a Pi2.

# General notes for myself

- Changed the timezone and local to Adelaide etc.
- Changed the default password and samba password, still using openhabian as the user
- Configured mosquitto as the MQTT broker
- Tested the mqtt server using MQTT.fx on windows

- Assigned the pi to 192.168.1.100 via DHCP reservation.
- Port 8080 has the webUI for OpenHAB.
- Mount the samba share and edit config files as required. WebUI seems painful to use.
