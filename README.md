[![Build Status](https://travis-ci.org/USC-ACTLab/crazyswarm.svg?branch=master)](https://travis-ci.org/USC-ACTLab/crazyswarm)

# crazyswarm
A Large Nano-Quadcopter Swarm.

The documentation is available here: http://crazyswarm.readthedocs.io/en/latest/.

### Not found in the official documentation
There are a few things that are not found in the official documentation. They are discussed below.

#### PC Permissions

PC permissions are discussed by Bitcraze and in [Flying Multiple UAVs Using ROS](http://act.usc.edu/publications/Hoenig_Springer_ROS2017.pdf), however, this package does not discuss it and does not automate it.

To add PC permissions, you can run the `pc_permissions.sh` script. You only need to run this once (if successful). You should log out/restart once this script executes.

#### Using Ubuntu Linux 18.04LTS with ROS Melodic
While the package is developed for Ubuntu Linux 16.04 and ROS Kinetic, a major portion of the package is compatible with Ubuntu Linux 18.04 and ROS Melodic. The observed problems and their solutions are as follows:
- Problem with the Qualisys2Ros package:

  When `build.sh` is run and the Qualisys2Ros package is cloned, you will probably get an error in `RTProtocol.cpp`. One solution is to open   `crazyswarm/ros_ws/src/externalDependencies/libmotioncapture/externalDependencies/Qualisys2Ros/include/RTProtocol.cpp` and replace line 451 with:
  ```
  if ((bool)ReceiveRTPacket(eType, false) == false)
  ```

  You can now run `build.sh` again and you should not get any errors.

- Problem with flashing the firmware:

  The arm compiler used for these packages is unstable/unsupported in Ubuntu 18.04. To flash the firmware please refer to `virtual_machine_files/README.md`.
