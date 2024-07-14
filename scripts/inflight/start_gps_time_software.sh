#!/bin/bash

# Restart daemons for turning gps inputs into disciplined time

# set by udev rules in 91-serial.rules
GPS_DEVICE=/dev/ttyGPS
# version controlled and uploaded to flight computer
CHRONY_CONF=/etc/chrony/chrony.conf

systemctl stop gpsd;
systemctl restart chronyd;
sleep 2;
systemctl restart gpsd;

exit 0
