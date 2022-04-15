#!/bin/bash

HR="----------------------------------------------------------------------"

check_retval () {
    if [ $1 -eq 0 ]; then
        echo $HR
        echo PASS
        echo $HR
    else
        exit $1
    fi
}

echo $HR
echo "Attempting groundhog build..."
pwd
echo $HR
cd ../groundhog
mkdir build
cd build && \
cmake .. && \
cmake --build .;
check_retval $?
cd ../..

echo $HR
echo "Attempting mole build..."
pwd
echo $HR
cd mole
mkdir build
cd build && \
cmake .. && \
cmake --build .;
check_retval $?
cd ../..

echo $HR
echo "Attempting guaca build..."
pwd
echo $HR
cd guaca
./configure && \
make;
check_retval $?
cd ..

echo $HR
echo "Attempting cow build..."
pwd
echo $HR
cd cow
./configure && \
make;
check_retval $?
cd ..;