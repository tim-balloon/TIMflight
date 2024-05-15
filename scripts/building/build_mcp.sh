#/bin/bash

cd mcp;
mkdir build;
cd build && cmake .. -DENABLE_TESTING=ON -DENABLE_STYLE_CHECK=ON -DCMAKE_BUILD_TYPE=Debug && cmake --build . && cd ../../