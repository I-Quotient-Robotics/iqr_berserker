#!/bin/bash

echo "delete remap the device serial port(ttyUSB*) to  berserker"
echo "sudo rm   /etc/udev/rules.d/arduino_due.rules"
sudo rm   /etc/udev/rules.d/arduino_due.rules
echo "delete remap the device serial port(ttyUSB*) to  jy901_gps"
echo "sudo rm   /etc/udev/rules.d/HL-340.rules"
sudo rm   /etc/udev/rules.d/HL-340.rules
echo "delete remap the device serial port(ttyUSB*) to  rplidar"
echo "sudo rm   /etc/udev/rules.d/rplidar.rules"
sudo rm   /etc/udev/rules.d/rplidar.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
