#!/bin/bash

path_to_repo=~/shubh/TIMflight/

cd $path_to_repo/kst/tim/
./all_kst.sh &
cd $path_to_repo/owl 
./owl ./owl-files/tim/tim2024.owl &
./owl ./owl-files/tim/motor*.owl &
./owl ./owl-files/tim/pointing.owl &
cow localhost &

