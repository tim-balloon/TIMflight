#!/bin/bash

# Completely halt infinite loop intended to restart mcp continuously

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

STOP_CMD="sudo pkill blast_inf_loop; sudo kill -INT \$(pidof mcp)"

ssh -t fc1user@$fc1_ip "$STOP_CMD"
ssh -t fc1user@$fc2_ip "$STOP_CMD"
