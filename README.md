pxpincher_ros Package
========================

This metapackage is intended for controlling and simulating the PhantomX Pincher robot at the RST.

Build status of the *master* branch (ROS Indigo):

[![Build Status](https://travis-ci.org/rst-tu-dortmund/pxpincher_ros.svg?branch=master)](https://travis-ci.org/rst-tu-dortmund/pxpincher_ros)

Build status of the *devel* branch (ROS Indigo):

[![Build Status](https://travis-ci.org/rst-tu-dortmund/pxpincher_ros.svg?branch=devel)](https://travis-ci.org/rst-tu-dortmund/pxpincher_ros)


Installation
------------

First, make sure that ROS is installed properly (we are currently on ROS indigo).
This package utilizes [ros_control](http://wiki.ros.org/ros_control) for controlling the robot:
    
    sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers


Now it is time to checkout this package:

    cd ~/catkin_ws/src
    git clone https://github.com/rst-tu-dortmund/pxpincher_ros.git


Check if everything compiles:

    cd ~/catkin_ws
    catkin_make
    
If you have unmet dependencies, try to install them using *rosdep*:

     cd ~/catkin_ws
     rosdep install --from-paths src --ignore-src --rosdistro indigo -y

Getting Started
---------------

### Hardware Bringup ###

The *pxpincher_hardware* and the communication interface *pxpincher_comm* provide a joint trajectory action server
that can be utilized to control the PhantomX Pincher robot arm. In case of dry experiments, a simulation mode is implemented in order
to mimic the actual joint motions.
 
If you want to control the real robot:
    
    
    roslaunch pxpincher_launch pxp.launch

In case of a simulation launch:
 
    roslaunch pxpincher_launch pxp_sim.launch

Issues:
 - Set permissions of the usb/serial port (only required if the unix user is not part of the system group *dialout*):
	
         sudo chmod 777 /dev/ttyUSB0

### Robot Visualization (RVIZ) ###

Run

    roslaunch pxpincher_launch pxp_rviz.launch


### Pxpincher Library (C++) ###

After starting the pxpincher hardware node (see [Hardware Bringup](#hardware-bringup)), the robot can be controlled using any suited action client.
However, you can utilize our C++ library to control the robot. Internally, the library implements an action client but provides many functionalities
including joint control, task space control, kinematics, etc.

Just add *pxpincher_lib* as build dependency to your node.

See an example usage in `pxpincher_lib/src/main.cpp`:

     rosrun pxpincher_lib pxpincher_test
 

### Utilities ###

The *pxpincher_lib* also provides a teaching node that can be used to print and modify joint configurations in RVIZ.
Additionally, joints of the actual robot can be relaxed in order to guide the robot towards a desired joint configuration manually.

Make sure that the *pxpincher_hardware* node is running (see [Hardware Bringup](#hardware-bringup)) and start:

     rosrun pxpincher_lib teach


License
-------
The *pxpincher_ros* meta-package is mainly developed and composed for educational purposes.

The individual packages (as part of the meta-package) are licensed under the BSD license.
The packages depend on other ROS packages, which are listed in the package.xml and that are also BSD licensed,
and the following third-party packages:
 * Eigen, MPL2 license, http://eigen.tuxfamily.org
 * CerealPort by Gonçalo Cabrita and Pedro Sousa, BSD license, http://wiki.ros.org/cereal_port, (here inlcuded in *pxpincher_comm*)

In order to simplify the configuration process, especially for educational purposes,
some modified versions of the *turtlebot_arm* packages (https://github.com/corot/turtlebot_arm, https://github.com/turtlebot/turtlebot_arm) are included, modified and renamed (*pxpincher_description*).



All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

Remarks
-------

In visualization, the separation of both fingers of the gripper is just a linear approximation of the actual euclidean distance. However, this is ok for our purposes
right now.

Note, this package includes an (slighly) adopted version of the turtlebot_arm package (resp. the branch https://github.com/corot/turtlebot_arm.git).
It is copied into this metapackage in order to allow a simple integration for students that start working with the PhantomX.
