#!/bin/bash

echo "remap the device serial port(ttyACM*) to  berserker"
echo "start copy arduino_due.rules to  /etc/udev/rules.d/"
sudo cp ./rules/arduino_due.rules  /etc/udev/rules.d
echo "remap the device serial port(ttyUSB*) to  jy901_gps"
echo "start copy HL-340.rules to  /etc/udev/rules.d/"
sudo cp ./rules/HL-340.rules  /etc/udev/rules.d
echo "remap the device serial port(ttyUSB*) to  rplidar"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
sudo cp ./rules/rplidar.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
