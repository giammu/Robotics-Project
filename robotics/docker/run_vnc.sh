#!/bin/bash
docker run -d --rm --net=ros \
   --env="DISPLAY_WIDTH=1920" --env="DISPLAY_HEIGHT=1080" --env="RUN_XTERM=no" \
   --name=novnc -p=8080:8080 \
   theasp/novnc:latest
