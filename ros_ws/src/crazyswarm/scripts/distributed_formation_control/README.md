# Distributed Formation Control Package for Crazyswarm
This package is designed to utilize a team of Crazyflie quadrotors through the Crazyswarm package to perform 3D formation control experiments.
While this `distributed_formation_control` folder does contain the required files to compute the required gains matrix given a desired formation and a sensing topology, the package requires other files in the `<ros_ws>/src/crazyswarm` directory to apply the desired formation control law.

## Dependencies
The `distributed_formation_control` package is not self contained for various reasons including certain file path requirements and the desire to keep the package as modular as possible. For that, the `distributed_formation_control` package requires the following components to function:
- `crazyswarm` package:
  The Crazyswarm server is required to control and track the drones over time. The `distributed_formation_control` package uses the topics provided by the Crazyswarm server to get global location information as well as issue the desired control commands.
- `<ros_ws>/src/crazyswarm/scripts/crazyflie_helper_functions`:
  The `crazyflie_helper_functions` package provides modular functions and tools developed at UTDallas that could be used for a variety of other applications. This package includes utilities such as the `distributed_collision_avoidance.py` file which provides the distributed collision avoidance required for the `distributed_formation_control` package.
- `<ros_ws>/src/crazyswarm/scripts/distributed_3d_form_ctrl_goto.py` and `<ros_ws>/src/crazyswarm/scripts/distributed_3d_form_ctrl_full_state.py`:
  These two scripts are independent and provide two ways of performing the distributed formation control algorithm. `distributed_3d_form_ctrl_goto.py` performs the algorithm, discritizes the velocity control vector and issues the command using the crazyswarm `goto` function. The `distributed_3d_form_ctrl_full_state.py` file performs the same algorithm but instead publishes a `full_state` message to the drones containing the full desired state of the drone (pose and derivative of pose). The latter consumes more of the radio bandwidth but might be more favorable for performance.

## ICRA 2019 Experiments
The experiments used for our "Robust 3D Distributed Formation Control with Collision Avoidance and Application to Multirotor Aerial Vehicles" ICRA 2019 paper were generated through this package. Below we discuss our implementation.

#### Hardware Implementation
On the hardware side, our implementation consisted of the following components:
- VICON motion capture system:
  Since the Crazyflie drones cannot perform the relative neighbor localization themselves, due to the lack of sensors and computational power, using an external motion capture system to locate the robots was inevitable. However, once the global positions are found, the local position measurements relative to each agent are generated and used in the process of calculating the control signal.
- Crazyflie quadrotors:
  The number of quadrotors used depends on the size of the formation. We tested out formations with 4, 5, 6, and 8 drones. As the number of drones increase, the rate at which the main loop runs becomes slower primarily due to the time it takes to issue control commands the drones.
- Crazyradio radio transmitters:
  We used 3 Crazyradio transmitters. The radios are customized to work with the Crazyflie drones. They handle transferring all data between the central computer and the drones.
- Central computer:
  The Crazyswarm package is a centralized package meaning that it requires a central computer to control all the drones and operate them. The computer we used for the experiments is a Dell Precision 7520 laptop with an Intel Xeon E3-1535M processor running at 3.10GHz.
  The computer uses the radios to communicate with the drones and gets the VICON data from a computer dedicated to run the VICON motion capture system.

#### Software Implementation
To generate the data, the following files were used:
- `<ros_ws>/src/crazyswarm/scripts/distributed_formation_control/formations.py`:
  This files defines the formations that were used in an arbitrary coordinates frame.
- `<ros_ws>/src/crazyswarm/scripts/crazyflie_helper_functions/distributed_collision_avoidance.py`:
  This files performs the distributed collision avoidance discussed in the paper.
- `<ros_ws>/src/crazyswarm/scripts/distributed_formation_control/matlab`:
  Contains Matlab code to calculate the gains matrix
- `<ros_ws>/src/crazyswarm/scripts/distributed_3d_form_ctrl_goto.py` and `<ros_ws>/src/crazyswarm/scripts/distributed_3d_form_ctrl_full_state.py`:
  These scripts perform the task of applying the formation control law through the following steps:
  - Calculating the gains matrix for a desired formation and sensing topology
  - Finding the current global location of the Crazyflies
  - Transforming the positions to local relative coordinates (for every agent, the location of its neighbors relative to itself are calculated)
  - Using the local coordinates to generate the control signal
  - Mapping the control signals back to the global frame
  - Issuing the control signal

   
