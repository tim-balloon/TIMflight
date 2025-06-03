#!/bin/bash

# Take scripts from local source dir and upload to both FCs. Doesn't restart.

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

SCRIPT_SRC=scripts/inflight/

LOOP=blast_inf_loop
GPS=start_gps_time_software.sh
FSW=start_flight_software.sh

# remote locations on the flight computers
REMOTE_TMP_DIR="/tmp"
REMOTE_INSTALL_DIR="/usr/local/sbin/"

# commands to run remotely on the flight computers
INSTALL_INF_LOOP="sudo install -m 755 -p $REMOTE_TMP_DIR/$LOOP $REMOTE_INSTALL_DIR"
INSTALL_START_GPS="sudo install -m 755 -p $REMOTE_TMP_DIR/$GPS $REMOTE_INSTALL_DIR"
INSTALL_START_FSW="sudo install -m 755 -p $REMOTE_TMP_DIR/$FSW $REMOTE_INSTALL_DIR"

echo "FC1:"
echo "Uploading critical runtime scripts..."
echo $LOOP
rsync -avz --rsync-path="sudo rsync" --delete $SCRIPT_SRC/$LOOP fc1user@$fc1_ip:$REMOTE_TMP_DIR
echo $GPS
rsync -avz --rsync-path="sudo rsync" --delete $SCRIPT_SRC/$GPS fc1user@$fc1_ip:$REMOTE_TMP_DIR
echo $FSW
rsync -avz --rsync-path="sudo rsync" --delete $SCRIPT_SRC/$FSW fc1user@$fc1_ip:$REMOTE_TMP_DIR
echo "Installing critical runtime scripts..."
echo $LOOP
ssh -t fc1user@$fc1_ip "$INSTALL_INF_LOOP;"
echo $GPS
ssh -t fc1user@$fc1_ip "$INSTALL_START_GPS;"
echo $FSW
ssh -t fc1user@$fc1_ip "$INSTALL_START_FSW;"

echo "FC2:"
echo "Uploading critical runtime scripts..."
echo $LOOP
rsync -avz --rsync-path="sudo rsync" --delete $SCRIPT_SRC/$LOOP fc1user@$fc2_ip:$REMOTE_TMP_DIR
echo $GPS
rsync -avz --rsync-path="sudo rsync" --delete $SCRIPT_SRC/$GPS fc1user@$fc2_ip:$REMOTE_TMP_DIR
echo $FSW
rsync -avz --rsync-path="sudo rsync" --delete $SCRIPT_SRC/$FSW fc1user@$fc2_ip:$REMOTE_TMP_DIR
echo "Installing critical runtime scripts..."
echo $LOOP
ssh -t fc1user@$fc2_ip "$INSTALL_INF_LOOP;"
echo $GPS
ssh -t fc1user@$fc2_ip "$INSTALL_START_GPS;"
echo $FSW
ssh -t fc1user@$fc2_ip "$INSTALL_START_FSW;"