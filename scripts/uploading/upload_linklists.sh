#!/bin/bash

# Take linklists from local source tree and upload to both FCs
LOCAL_LINKLIST_SRC=blast_config/linklists/
LOCAL_LINKLIST_DEST=/data/etc/linklists/
REMOTE_LINKLIST_DEST=/data/etc/linklists/

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4
main_gse_ip=192.168.1.223

echo "This GSE:"
echo "Installing linklists..."
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC/*.ll* $LOCAL_LINKLIST_DEST;

echo "TIM main GSE:"
echo "Installing linklists..."
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC/*.ll* tim@$main_gse_ip:$REMOTE_LINKLIST_DEST;

echo "FC1:"
echo "Installing linklists..."
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC/*.ll* fc1user@$fc1_ip:$REMOTE_LINKLIST_DEST;

echo "FC2:"
echo "Installing linklists..."
<<<<<<< HEAD
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC*.ll* fc1user@$fc2_ip:$REMOTE_LINKLIST_DEST
=======
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC*.ll* fc1user@$fc2_ip:$REMOTE_LINKLIST_DEST
>>>>>>> 13d9d5bdca6e4857513e88c3fd22509745590732
