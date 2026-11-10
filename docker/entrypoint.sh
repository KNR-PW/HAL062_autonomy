#!/bin/bash
set -e

echo "|"
echo "| + HAL062 - Isaac ROS dev container"
echo "| + Release: 5.0"
echo "|"

# Trick for host-docker ownership issue
export USERNAME="${USERNAME:-hal062-admin}"
export HOST_USER_UID="${HOST_USER_UID:-1000}"
export HOST_USER_GID="${HOST_USER_GID:-1000}"
chown "${HOST_USER_UID}:${HOST_USER_GID}" "${HAL062_WS}"

exec /usr/local/bin/scripts/workspace-entrypoint.sh "$@"
