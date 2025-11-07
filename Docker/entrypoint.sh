#!/bin/bash
set -e

# set up ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"