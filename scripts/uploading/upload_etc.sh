#!/bin/bash

# Take etc config files from local source tree and upload to both FCs. Doesn't restart any daemons

ETC_SRC=blast_etc/

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/chrony/chrony.conf fc1user@$fc1_ip:/etc/chrony/
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/chrony/chrony.conf fc1user@$fc2_ip:/etc/chrony/

rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/systemd/flight_software.service fc1user@$fc1_ip:/etc/systemd/system/
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/systemd/flight_software.service fc1user@$fc2_ip:/etc/systemd/system/

rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/udev/91-serial.rules fc1user@$fc1_ip:/etc/udev/rules.d/
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/udev/91-serial.rules fc1user@$fc2_ip:/etc/udev/rules.d/
