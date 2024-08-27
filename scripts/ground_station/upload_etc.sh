#!/bin/bash

# Take etc config files from local source tree and upload to both FCs. Doesn't restart any daemons

separator="-------------------------------------------------------------------------------"
ETC_SRC=blast_etc/

gse_ip=192.168.1.223

GSE_LUT_DIR="/data/etc/blast/"
# array of lookup tables
declare -a LUTS=(
                 "thermistor_R_to_T_TIM.lut"
                 "thermistor_V_to_R_TIM.lut"
                )

echo $separator
echo "GSE:"
echo $separator
echo "Uploading lookup tables..."
for item in "${LUTS[@]}"
    do
        rsync -avz --rsync-path="sudo rsync" --delete $ETC_SRC/$item tim@$gse_ip:$GSE_LUT_DIR
    done
