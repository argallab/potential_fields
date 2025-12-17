#!/bin/bash

# Get the absolute path to the project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================================"
echo "1. Building Docker Image 'pfields_2025'"
echo "================================================================"
# Build the image using the Dockerfile in Docker/ folder
# Context is the project root
docker build -t pfields_2025 -f "$PROJECT_ROOT/Docker/Dockerfile" "$PROJECT_ROOT"

echo ""
echo "================================================================"
echo "2. Preparing to Run Container"
echo "================================================================"
echo " - Mounting workspace: $PROJECT_ROOT -> /home/workspace/src/pfields_2025"
echo " - Enabling X11 forwarding (for GUI apps like Rviz)"

# Allow local connections to X server (be careful with security on public networks)
xhost +local:docker > /dev/null 2>&1

echo ""
echo "================================================================"
echo "3. Starting Container"
echo "================================================================"
echo "NOTE: The container will automatically build 'potential_fields_library' on startup."
echo "      Wait for 'potential_fields_library successfully installed' before typing."
echo ""

docker run -it --rm \
    --net=host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PROJECT_ROOT:/home/workspace/src/pfields_2025" \
    pfields_2025

# Example usage inside container:
# $ colcon build --packages-select potential_fields
# $ source install/setup.bash
# $ ros2 launch potential_fields ...
