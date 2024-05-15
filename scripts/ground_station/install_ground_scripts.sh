#!/bin/bash

# Install scripts into ground station bin

INSTALL_DIR=/usr/local/sbin/

sudo ln -s $(pwd)/install_and_restart_mcp.sh $INSTALL_DIR 
sudo ln -s $(pwd)/restart_blastgs.sh $INSTALL_DIR 
