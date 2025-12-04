#!/bin/bash

# Run this script to build the pfield_library

echo "Building pfield_library..."

# Create build directory and run cmake + make
cd "pfield_library/"
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/pfield_libs_install ..
make -j4
make install
echo -e "\e[32mpfield_library built and installed to $HOME/pfield_libs_install\e[0m"
cd -
