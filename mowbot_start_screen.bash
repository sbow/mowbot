#!/bin/bash
# Start Luma python script for  128 x 64 i2c display on bootup
# 
# Shaun Bowman, 2018 / 09 / 30
#
# NVidia Jetson Tx1
# GPIO Pin 29 - gpio219
# reference: https://www.jetsonhacks.com/nvidia-jetson-tx1-j21-header-pinout/
#
# Adafruit 128 x 64 oled monochrome display
# i2c address: 0x3d (may be 0x3c for some displays)
# to scan for addresses:
# sudo i2cdetect -r -y 1
# where 1 is i2c port number
# sudo i2cdump -y 1 0x3d
# ^ you can see information form device, not usefull in this case
#
# To run as root on bootup:
# chmod 755 mowbot_start_screen.bash
# sudo cp mowbot_start_screen.bash /etc/init.d
# ls /etc/rc2.d
# (see highest number, say S15)
# sudo ln -s /etc/init.d/mowbot_start_screen.bash /etc/rc2.d/S16mowbot_start_screen.bash
# (reboot)
#
# Note: 2 k resitor is attached to reset pin of OLED & pin 29.
# this script pulls the pin high low high
# This resets the luma OLED.
# If you run a luma script without reseting, the OLED may display "snow"/garbage.
# Using the resetor & this script eliminates this problem

cd /sys/class/gpio
echo 219 > export
cd gpio219
echo out > direction && echo 1 > value
sleep 1
echo out > direction && echo 0 > value
sleep 1
echo out > direction && echo 1 > value
cd /home/nvidia/git/luma.examples/examples 
cmd="python mowbot.py --i2c-address 0x3d"
nohup $cmd &
#disown -h $cmd &
cd /home/nvidia

