#!/bin/bash

# Path to virtual_machine_files
ROOT=$PWD

sudo groupadd plugdev
sudo usermod -a -G plugdev $USER

# Write/Overwrite the rules files
echo -e "#Crazyradio (normal operation) \nSUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"7777\", MODE=\"0664\", GROUP=\"plugdev\" \n# Bootloader \nSUBSYSTEM==\"usb\", ATTRS{idVendor}==\"1915\", ATTRS{idProduct}==\"0101\", MODE=\"0664\", GROUP=\"plugdev\"" | sudo tee /etc/udev/rules.d/99-crazyradio.rules

echo -e "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5740\", MODE=\"0664\", GROUP=\"plugdev\"" | sudo tee /etc/udev/rules.d/99-crazyflie.rules

# Reload rules
sudo udevadm control --reload-rules && udevadm trigger

# Go back to original directory
cd $ROOT

# Go to crazyflie-firmware
cd ../crazyflie-firmware/

# Download necessary files
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi -y

# Compile code
make clean; make

# Go to crazyflie2-nrf-firmware
cd ../crazyflie2-nrf-firmware/

# Compile code
make clean; make

# Go to crazyflie-client-python
cd ../crazyflie-clients-python/

# Download required files
sudo apt-get install python3 python3-pip python3-pyqt5 -y
pip3 install -e .

# Go back to original directory
cd $ROOT
