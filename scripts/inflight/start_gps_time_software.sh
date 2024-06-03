#!/bin/bash

# Restart daemons for turning gps inputs into disciplined time

# set by udev rules in 91-serial.rules
GPS_DEVICE=/dev/ttyGPS
# version controlled and uploaded to FC
CHRONY_CONF=/etc/chrony/chrony.conf

if pgrep -x "chronyd" > /dev/null
then
    killall chronyd
    echo "chronyd process killed"
else
    echo "chronyd started"
fi

if pgrep -x "gpsd" > /dev/null
then
    killall gpsd
    echo "gpsd process killed"
else
    echo "gpsd started"
fi

# restart chrony and gpsd
chronyd -f $CHRONY_CONF
gpsd -n $GPS_DEVICE

exit 0