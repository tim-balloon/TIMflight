#! /bin/bash

# Issue commands to kill and restart blastcmd daemon on FCs

fc1_ip="192.168.1.3"
fc2_ip="192.168.1.4"

# remote locations on the flight computers
REMOTE_INSTALL_DIR="/usr/local/sbin"

# commands to run remotely on the flight computers
KILL_BLASTCMD="sudo kill -INT \$(pidof blastcmd) > /dev/null 2>&1"
START_BLASTCMD="sudo $REMOTE_INSTALL_DIR/blastcmd -d -fifo > /dev/null 2>&1"

# restart new executables.
ssh -t fc1user@$fc1_ip "$KILL_BLASTCMD; $START_BLASTCMD;"
ssh -t fc1user@$fc2_ip "$KILL_BLASTCMD; $START_BLASTCMD;"
