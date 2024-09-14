#! /bin/sh

for kstfile in $(ls *.kst)
    do
        kst2 $kstfile &
    done