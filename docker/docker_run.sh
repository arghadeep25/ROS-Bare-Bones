#!/bin/bash
docker run --net=host -it --rm \
           -v $(realpath ..):/root/catkin_ws/src/rosbarebones \
           -w /root/catkin_ws/src/rosbarebones \
           $@ \
           rosbarebones