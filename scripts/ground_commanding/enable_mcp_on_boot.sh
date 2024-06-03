#!/bin/bash

# Issue commands to kill and restart blast_inf_loop on FCs

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

# remote locations on the flight computers
REMOTE_INSTALL_DIR="/etc/systemd/system/"

ENABLE_FSW_SERVICE="sudo chmod +x $REMOTE_INSTALL_DIR/flight_software.service; sudo systemctl enable flight_software.service"

# commands to run remotely on the flight computers
ssh -t fc1user@$fc1_ip "$ENABLE_FSW_SERVICE;"
ssh -t fc1user@$fc2_ip "$ENABLE_FSW_SERVICE;"


