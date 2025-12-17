#!/bin/bash

# Run this script to build the potential_fields_library

echo "Building potential_fields_library..."

# Build using colcon
colcon build --packages-select potential_fields_library

echo -e "\e[32mpotential_fields_library built\e[0m"
