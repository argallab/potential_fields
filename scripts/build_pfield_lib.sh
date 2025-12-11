#!/bin/bash

# Run this script to build the pfield_library

echo "Building pfield_library..."

# Build using colcon
colcon build --packages-select pfield_library

echo -e "\e[32mpfield_library built\e[0m"
