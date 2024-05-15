#!/bin/bash

# Run on FC boot. Start everything needed for flight operations.

# GPS + time server
# echo "Starting gpsd and chrony"
# /usr/local/sbin/restart_gps_time_software.sh

# blastcmd
echo "Starting blastcmd"
/usr/local/sbin/blastcmd -d -fifo

# mcp infinite loop
echo "Starting blast_inf_loop"
sudo /usr/local/sbin/blast_inf_loop