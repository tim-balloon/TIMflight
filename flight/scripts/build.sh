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
echo "Attempting mcp build..."
pwd
echo $HR
cd ../mcp
mkdir build
cd build &&
cmake .. &&
cmake --build .;
check_retval $?
