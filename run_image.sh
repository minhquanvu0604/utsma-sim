#!/bin/bash

docker run --privileged \
           --detach \
           --restart 'always' \
           --volume ${PWD}:/root/utsma_ws/src/utsma-sim \
           --name utsma_dev_container \
           utsma_dev tail -f /dev/null