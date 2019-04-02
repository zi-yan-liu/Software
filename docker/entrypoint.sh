#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

source "/home/software/docker/env.sh"

exec "$@"
