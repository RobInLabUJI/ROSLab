#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/lunar/setup.bash"
exec "$@"
