#!/bin/bash

cd blastcmd;
echo $(pwd)
./bootstrap.sh &&
./configure &&
make &&
sudo make install;