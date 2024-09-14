#/bin/bash

cd mcp;
mkdir build;
cd build && cmake .. -DENABLE_TESTING=ON -DENABLE_STYLE_CHECK=ON -DENABLE_ADDRESS_SANITIZER=OFF -DCMAKE_BUILD_TYPE=Release && cmake --build . -j && cd ../../
