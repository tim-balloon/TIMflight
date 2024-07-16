#!/bin/bash

# Take etc config files from local source tree and upload to both FCs. Doesn't restart any daemons

separator="-------------------------------------------------------------------------------"
ETC_SRC=blast_etc/

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4
sc1_ip=192.168.1.137
sc2_ip=192.168.1.138

echo $separator
echo "FC1:"
echo $separator
echo "Uploading chrony config..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/chrony/chrony.conf fc1user@$fc1_ip:/etc/chrony/
echo "Uploading netplan config..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/netplan/00-installer-config_fc1.yaml fc1user@$fc1_ip:/etc/netplan/
echo "Uploading flight software service..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/systemd/flight_software.service fc1user@$fc1_ip:/etc/systemd/system/
echo "Uploading udev rules..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/udev/91-serial.rules fc1user@$fc1_ip:/etc/udev/rules.d/

echo $separator
echo "FC2:"
echo $separator
echo "Uploading chrony.conf..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/chrony/chrony.conf fc1user@$fc2_ip:/etc/chrony/
echo "Uploading netplan config..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/netplan/00-installer-config_fc2.yaml fc1user@$fc2_ip:/etc/netplan/
echo "Uploading flight software service..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/systemd/flight_software.service fc1user@$fc2_ip:/etc/systemd/system/
echo "Uploading udev rules..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/udev/91-serial.rules fc1user@$fc2_ip:/etc/udev/rules.d/

echo $separator
echo "SC1:"
echo $separator
echo "Uploading chrony.conf..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/chrony/starcam/chrony.conf starcam@$sc1_ip:/etc/chrony/
echo "Uploading netplan config..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/netplan/00-installer-config_sc1.yaml starcam@$sc1_ip:/etc/netplan/

echo $separator
echo "SC2:"
echo $separator
echo "Uploading chrony.conf..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/chrony/starcam/chrony.conf starcam@$sc2_ip:/etc/chrony/
echo "Uploading netplan config..."
rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/netplan/00-installer-config_sc2.yaml starcam@$sc2_ip:/etc/netplan/
