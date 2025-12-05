#!/bin/bash

# Builds everything in the repository

# Run Colcon Build in the repository directory
# Builds all packages including pfield_library
colcon build

echo -e "\e[32mRepository build complete.\e[0m"

# Source the install setup.bash
source install/setup.bash

echo -e "\e[32mSourced the workspace install/setup.bash\e[0m"
echo -e "\e[32mFinished Building pfields repository\e[0m"
