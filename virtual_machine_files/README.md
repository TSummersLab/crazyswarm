# Virtual Machine files

Using Ubuntu 18.04 to run this package is fine for the most part. However, there are a few issues when it comes to flashing the firmware. As of October 2018, the arm cross-assembly package for Linux 18.04 does not work properly with crazyflie-firmware or crazyflie2-nrf-firmware. In addition, using the Bitcraze VM might prove troublesome as there are some missing directories.

The easiest way to flash the firmware on the crazyflie quadrotors is by installing a virtual machine (VM) with Linux 16.04 and compiling the firmware through this package.

Follow these steps:
1. Download and install [Oracle VirtualBox](https://www.virtualbox.org/wiki/Downloads)(VB). Any version should be fine.
2. Through the [same link](https://www.virtualbox.org/wiki/Downloads), download the VirtualBox Extension Pack (for all supported platforms). Run the software once VB has been installed
3. Get a Linux 16.04 desktop image. You may use [this link](http://releases.ubuntu.com/16.04/) to do that.
4. Create a new virtual machine. There are many resources on how to do that including [this](https://docs.oracle.com/cd/E26217_01/E26796/html/qs-create-vm.html).
5. Start your virtual machine and follow through the installation process. Once done, allow the virtual machine to restart.
6. In your VM, open a new terminal window and run the following:
```
sudo apt-get install git -y
cd ~
git clone https://github.com/The-SS/crazyswarm.git
cd crazyswarm
./build.sh
cd virtual_machine_files
```
You can ignore all errors related to ROS, catkin, or similar things.
7. In that terminal run the following script `config_vm.sh` as a super user: `sudo ./config_vm.sh`. Follow with the script as you might have to enter your password or press enter or y (yes) for certain installs. This should take around 10-15 minutes.
8. It is a good idea to restart your VM (although steps to avoid having to do that have been applied in the script).
9. Add the radio as an input device to the VM: from the top menu bar go to `Devices --> USB` and select the `Bitcraze Crazyradio` option.
9. You can now use the cfclient to configure your drones as such:
```
cd ~/crazyswarm/crazyflie-clients-python
python3 bin/cfclient
```
You can also flash the crazyflie firmwares (STM and NRF) respectively as such:
```
cd ~/crazyswarm/crazyflie-firmware
make cload
```
```
cd ~/crazyswarm/crazyflie2-nrf-firmware
make cload
```


## Notes:
- If you have problems flashing the drones try running `make clean; make` to recompile the code.
