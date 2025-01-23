#!/bin/sh
if [ -d "build" ]; then
    rm -rf build
    echo "build directory cleaned"
fi
mkdir build
cd build
cmake ..
make -j$(nproc)
cd ..