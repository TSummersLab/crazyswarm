#!/bin/bash 

# Use to fix the issue with compiling crazyswarm in Ubuntu 18.04 due to Qualisys
# Update line 451 in `crazyswarm/ros_ws/src/externalDependencies/libmotioncapture/externalDependencies/Qualisys2Ros/include/RTProtocol.cpp` by adding a typecast

# Run in root of package (`crazyswarm`)

FILENAME="ros_ws/src/externalDependencies/libmotioncapture/externalDependencies/Qualisys2Ros/include/RTProtocol.cpp"

sed -i '/                if (ReceiveRTPacket(eType, false) == false)/c\
                if ((bool)ReceiveRTPacket(eType, false) == false)' $FILENAME
