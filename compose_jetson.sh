#!/bin/bash

docker_scripts/for_jetson/sudo docker-compose up &

echo 388 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio388/direction
echo 1  > /sys/class/gpio/gpio388/value

sleep 2

echo 0 > /sys/class/gpio/gpio388/value

sleep 1 

echo 1 > /sys/class/gpio/gpio388/value

sleep 1