# Go1-Full

Software package for Go1, using the Unitree SDK. This is a self-contained repository with correct versions to work with the current Go1 robots in the BioRob lab. New robots may have different SDKs. 

## Installation Notes

* Install ROS according to the appropriate [tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu).
Make sure to source, add to your `.bashrc` file

* Create a catkin workspace according to the following [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). If you are using a BIOROB pc, this should be in `/data/<USER_NAME>/`, and the CMakeLists assume you call it `catkin_ws_go1_full`, so for example this looks like `/data/bellegar/catkin_ws_go1_full/`. If you want to make use of the BIOROB backups, you can put your files in your home directory somewhere, and symbolically link to your catkin workspace `src/` directory.  

* Install ROS dependencies, for noetic (also detailed in [unitree_ros](/unitree_ros)). If you are using a BIOROB PC and do not have access to sudo, this is probably already done for you.
```
sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

* Install dependencies for [unitree_legged_sdk](/unitree_legged_sdk), for example LCM. Again, if you are using a BIOROB PC, this is done for you. However, you still need to follow the directions to build. 

* Download Torch (explained below) and place it in `/data/`. If you are using a BIOROB PC, this is probably done for you. It is not yet used in the code, but this functionality may be added soon. 

* Compile with `catkin_make -DCMAKE_BUILD_TYPE=Release` and test launch and run_cpg as detailed below.


## Dependencies
The following repositories should be in your `catkin_ws/src` directory (the following are the ORIGINAL links, where the working versions are now included in this repo): 
* [unitree_ros](https://github.com/unitreerobotics/unitree_ros) 
* [unitree_ros_to_real](https://github.com/unitreerobotics/unitree_ros_to_real)
* [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk)

Avoid rewriting code for Gazebo/hardware - ideally only the exectuable should change. 

To run the Bayesian optimization, you need to install `optuna`, `cma`, etc. in a virtual environment, and activate in the appropriate terminal [todo add specifics].

### Torch
Policies trained in Isaac Gym can be written out and tested both in Gazebo and on the hardware. 

* Make sure to download the cxx11 ABI version of libtorch:
https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-1.10.0%2Bcpu.zip

* Verify you can link properly by following this tutorial:
https://pytorch.org/tutorials/advanced/cpp_export.html

## Adding New Functionalities

The intended use is to create a directory in [go1_software](/go1_software) with a new "skill", such as "Balance", "MPC", "HRI" etc. Then, create executables in the same spirit as [run_cpg.cpp](/go1_software/src/exe/run_cpg.cpp) and [run_cpg_lcm.cpp](/go1_software/src/exe/run_cpg_lcm.cpp) to first test in Gazebo, and then on the hardware.


## Gazebo

### Run CPG or Jump Example

**Note**: there are two versions of go1 now: go1_old, and go1_new. The latter is a test based on the new files from unitree (not debugged yet), so use go1_old for now. 

* `roslaunch unitree_gazebo normal.launch rname:=go1_old`
* `rosrun go1_software run_cpg`
* `rosrun go1_software run_jump`

Bayesian Optimization: (make sure that initial conditions are same in `JumpingObj.h` and `optimize_jump.py`)
* `roslaunch go1_software optimize_jump.launch`
* `roslaunch go1_software optimize_jump_lcm.launch`

Use realsense camera for state estimation T265:
* `/data/bellegar/catkin_ws_go1_full/src/go1_full/go1_software$ ./main_script.sh`

Face lights:
* `unitree@nano2gb:~/software/faceLightSDK_Nano$ ./bin/faceLightClient`


## Human-Robot Interaction Example

Please check `TODO` tags in `go1_software/jumping/HRI.cpp`

* `rosrun go1_software hri`

This will first move the robot to a 3 foot on the ground configuration, and then start the handshake. You can also see an example of sending updated handshaking parameters in `go1_software/scripts/send_handshake_params.py`. 

The hardware interface is also implemented:

* `rosrun go1_software hri_lcm`


## Hardware (ONLY ALLOWED WITH PERMISSION)

**For low-level control:** CPG with LCM, converting to ROS for data processing. **Make sure the robot is in "basic mode"** (after turning on, L2+A twice to lay down, L2+B to enter damping mode, then L1+L2+B. Now, can return to standing with L2+B, L2+A). **IMPORTANT**: see notes at the top of [run_cpg_lcm.cpp](/go1_software/src/exe/run_cpg_lcm.cpp) on what is allowed (i.e. no torque control, no large gains, nothing near robot power/torque limits, etc.).

* `roslaunch unitree_legged_real real.launch ctrl_level:=lowlevel` (optional)
* `rosrun go1_software run_cpg_lcm`

**For high-level control:** make sure the robot is in sport mode (i.e. walking mode, with START, NOT lying down). There is an example to use Unitree's high level API to control base orientation and moving forward/backward/left/right at [example_walk_lcm.cpp](/go1_software/src/exe/example_walk_lcm.cpp)

* `roslaunch unitree_legged_real real.launch ctrl_level:=highlevel` (optional)
* `rosrun go1_software example_walk_lcm`

## Notes

* Foot sensors are very inaccurate. Code attempts to find "zero" (in air) value, and then scales by gravity (may not be correct, but values are more reasonable). It is important to have a good estimate of these since the state estimation depends on this. 
* LCM data returned from robot seems to be a subset of total sensor info (i.e. compared to ROS msgs). 

## Resources

* [Unitree support youtube channel](https://www.youtube.com/channel/UCUyDbokbR3MhWo-GR1ansBQ?app=desktop)
* [Go1 Documentation from Unitree](https://drive.google.com/drive/u/2/folders/1QWYfP1b7W4IWBd1JxT-wUMzFepR2F7u9)


## TODO

* Integrate python interface.
* Clean up CMakeLists

## Possible Issues

* If the simulation doesn't run well and the time factor is quite low, change the real_time_update_rate to something smaller based on your computer performance (e.g 100) in the *.world file (i.e. [earth.world](/unitree_ros/unitree_gazebo/worlds/earth.world))

## Allow internet connection through PC

```
unitree@unitree-desktop:~$ cat /etc/network/interfaces
# interfaces(5) file used by ifup(8) and ifdown(8)
# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d
 
auto eth0
iface eth0 inet static
address 192.168.123.14
netmask 255.255.255.0
# gateway 192.168.123.1
gateway 192.168.123.29
dns-nameservers 128.178.15.8
```

Remove wrong sources..
``` 
unitree@unitree-desktop:~$ cat /etc/apt/sources.list
deb http://ports.ubuntu.com/ubuntu-ports/ bionic main multiverse restricted universe
deb http://ports.ubuntu.com/ubuntu-ports/ bionic-security main multiverse restricted universe
deb http://ports.ubuntu.com/ubuntu-ports/ bionic-updates main multiverse restricted universe
deb http://ports.ubuntu.com/ubuntu-ports/ bionic-backports main multiverse restricted universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic main multiverse restricted universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic-security main multiverse restricted universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic-updates main multiverse restricted universe
deb-src http://ports.ubuntu.com/ubuntu-ports/ bionic-backports main multiverse restricted universe
# deb http://security.ubuntu.com/ubuntu xenial-security main
# deb-src http://security.ubuntu.com/ubuntu xenial-security main
# deb http://ports.ubuntu.com/ubuntu-ports/ xenial restricted universe main multiverse
# deb-src http://ports.ubuntu.com/ubuntu-ports/ xenial restricted universe main multiverse
deb https://librealsense.intel.com/Debian/apt-repo bionic main
# deb-src https://librealsense.intel.com/Debian/apt-repo bionic main

```

Reset language/time
```
sudo dpkg-reconfigure tzdata
sudo dpkg-reconfigure locales

```


## Check and kill ROS processes still lingering from gazebo

`ps ax | grep ros` 