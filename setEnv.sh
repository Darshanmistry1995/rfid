#!/bin/bash
echo BB-UART1 > /sys/devices/bone_capemgr.*/slots
echo BB-UART2 > /sys/devices/bone_capemgr.*/slots
#echo BB-UART4 > /sys/devices/bone_capemgr.*/slots

echo STIV-GPIO > /sys/devices/bone_capemgr.*/slots
echo 49 > /sys/class/gpio/export
echo 115 > /sys/class/gpio/export
echo 60 > /sys/class/gpio/export
#echo 48 > /sys/class/gpio/export
echo 50 > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio115/direction
echo "out" > /sys/class/gpio/gpio49/direction
echo "out" > /sys/class/gpio/gpio60/direction
#echo "in" > /sys/class/gpio/gpio48/direction
echo "out" > /sys/class/gpio/gpio50/direction
