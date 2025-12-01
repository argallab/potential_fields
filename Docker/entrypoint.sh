#!/bin/bash
set -e

# Auto-build pfield_library if found
# Check common mount locations
POSSIBLE_PATHS=(
    "/home/workspace/src/pfields_2025/pfield_library"
    "/home/workspace/src/pfield_library"
    "/home/workspace/pfield_library"
)

for path in "${POSSIBLE_PATHS[@]}"; do
    if [ -d "$path" ]; then
        echo "Auto-detected pfield_library at $path"
        # Use the script we installed in the Dockerfile
        build_pfield_lib.sh "$path"
        break
    fi
done

# set up ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
