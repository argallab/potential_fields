#!/bin/bash

# Builds everything in the repository

# First, run the pfield_library build script
# Can set environment variable if needed
# export LD_LIBRARY_PATH=$HOME/pfield_libs_install/lib:$LD_LIBRARY_PATH
cd ${HOME}/final_project/pfields_2025

# if given -build_pf arg, then run the pfield_library build
if [[ "$1" == "-build_pf" ]]; then
    echo -e "\e[32mBuilding pfield_library...\e[0m"
  ./scripts/build_pfield_lib.sh
else
    echo -e "\e[33mSkipping pfield_library build. To build, run with -build_pf argument.\e[0m"
fi


# Run Colcon Build in the repository directory
colcon build --packages-select potential_fields potential_fields_interfaces pfields_demo \
  --cmake-args -DCMAKE_PREFIX_PATH=~/pfield_libs_install

echo -e "\e[32mRepository build complete.\e[0m"

# Source the install setup.bash
source install/setup.bash

echo -e "\e[32mSourced the workspace install/setup.bash\e[0m"
echo -e "\e[32mFinished Building pfields repository\e[0m"
