#!/bin/bash

# Take blastcmd from local build dir and upload to both FCs. Doesn't restart.

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

# newly-built executable locations
BLASTCMD_BUILD_DIR=blastcmd/
BLASTCMD_PATH=$BLASTCMD_BUILD_DIR/blastcmd

# remote locations on the flight computers
REMOTE_TMP_DIR="/tmp/"
REMOTE_INSTALL_DIR="/usr/local/sbin/"

# commands to run remotely on the flight computers
KILL_BLASTCMD="sudo kill -INT \$(pidof blastcmd) > /dev/null 2>&1"
INSTALL_BLASTCMD="sudo install -m 755 -p $REMOTE_TMP_DIR/blastcmd $REMOTE_INSTALL_DIR"

echo "FC1:"
echo "Uploading blastcmd..."
rsync -avz --rsync-path="sudo rsync" --delete $BLASTCMD_PATH fc1user@$fc1_ip:$REMOTE_TMP_DIR
echo "Halting and installing blastcmd..."
ssh -t fc1user@$fc1_ip "$KILL_BLASTCMD; $INSTALL_BLASTCMD;"

echo "FC2:"
echo "Uploading blastcmd..."
rsync -avz --rsync-path="sudo rsync" --delete $BLASTCMD_PATH fc1user@$fc2_ip:$REMOTE_TMP_DIR
echo "Halting and installing blastcmd..."
ssh -t fc1user@$fc2_ip "$KILL_BLASTCMD; $INSTALL_BLASTCMD;"