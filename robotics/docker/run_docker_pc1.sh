#!/bin/bash

CONTAINER_NAME="robotics_first_pc"

# Check if the container exists
if [ $(docker ps -a -q -f name=^/${CONTAINER_NAME}$) ]; then
    echo "Container '${CONTAINER_NAME}' already running. Removing it ..."
    docker rm -f $CONTAINER_NAME
    echo "Container '${CONTAINER_NAME}' has been removed."
else
    echo "Container '${CONTAINER_NAME}' does not exist."
fi

echo "Starting '${CONTAINER_NAME}' container."

MOUNT_PATH="/home/$USER/catkin_ws/src"
SRC_PATH="$(pwd)/../src"

# Start the container. 
docker run -it \
    --net=ros \
    --env="DISPLAY=novnc:0.0" \
    --volume="${SRC_PATH}:${MOUNT_PATH}" \
     --name=$CONTAINER_NAME \
    robotics:stable \
    /bin/bash 
