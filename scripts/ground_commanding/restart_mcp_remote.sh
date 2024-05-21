#!/bin/bash

# Issue commands to kill and restart blast_inf_loop on FCs

fc1_ip="192.168.1.3"
fc2_ip="192.168.1.4"

# remote locations on the flight computers
REMOTE_TMP_DIR="/tmp"
REMOTE_INSTALL_DIR="/usr/local/sbin"

# commands to run remotely on the flight computers
INF_LOOP_CHECK="cd; sudo pkill blast_inf_loop; sudo nohup $REMOTE_INSTALL_DIR/blast_inf_loop & > /dev/null 2> /dev/null < /dev/null"
KILL_MCP="sudo kill -INT \$(pidof mcp) > /dev/null 2>&1"

# bring down infinite mcp loop, kill any running mcps such that the new one is started by inf loop.
ssh -t fc1user@$fc1_ip "$INF_LOOP_CHECK; $KILL_MCP"
ssh -t fc1user@$fc2_ip "$INF_LOOP_CHECK; $KILL_MCP"
