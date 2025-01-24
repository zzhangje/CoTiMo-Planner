#!/bin/sh
if [ ! -d "build" ]; then
    mkdir build
    echo "build directory created"
fi
cd build
cmake ..
make -j$(nproc)
cd ..
./build/test