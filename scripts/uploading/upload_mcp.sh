#!/bin/bash

# Take mcp from local build dir and upload to both FCs. Does not restart.

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

# newly-built executable locations
MCP_BUILD_DIR=mcp/build/
MCP_PATH=$MCP_BUILD_DIR/mcp

# remote locations on the flight computers
REMOTE_TMP_DIR="/tmp/"
REMOTE_INSTALL_DIR="/usr/local/sbin/"

# commands to run remotely on the flight computers
INF_LOOP_CHECK="cd; sudo pkill mcp_inf_loop"
KILL_MCP="sudo kill -INT \$(pidof mcp) > /dev/null 2>&1"
INSTALL_MCP="sudo install -m 755 -p $REMOTE_TMP_DIR/mcp $REMOTE_INSTALL_DIR"

echo "FC1:"
echo "Uploading mcp..."
rsync -avz --rsync-path="sudo rsync" --delete $MCP_PATH fc1user@$fc1_ip:$REMOTE_TMP_DIR
echo "Halting mcp_inf_loop and installing mcp..."
ssh -t fc1user@$fc1_ip "$INF_LOOP_CHECK; $KILL_MCP; $INSTALL_MCP;"

echo "FC2:"
echo "Uploading mcp..."
rsync -avz --rsync-path="sudo rsync" --delete $MCP_PATH fc1user@$fc2_ip:$REMOTE_TMP_DIR
echo "Halting mcp_inf_loop and installing mcp..."
ssh -t fc1user@$fc2_ip "$INF_LOOP_CHECK; $KILL_MCP; $INSTALL_MCP;"