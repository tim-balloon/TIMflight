#!/bin/bash

# Take linklists from local source tree and upload to both FCs
LOCAL_LINKLIST_SRC=blast_config/linklists/
LOCAL_LINKLIST_DEST=/data/etc/linklists/
REMOTE_LINKLIST_DEST=/data/etc/linklists/

fc1_ip=192.168.1.3
fc2_ip=192.168.1.4

sudo rsync -avz --delete  $LOCAL_LINKLIST_DEST
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC fc1user@$fc1_ip:$REMOTE_LINKLIST_DEST;
rsync -avz --rsync-path="sudo rsync" --delete $LOCAL_LINKLIST_SRC fc1user@$fc2_ip:$REMOTE_LINKLIST_DEST
