# SharedROSNodes
ROS nodes we can reuse between project packages.

# Usage
This package targets ROS Kinetic.

1. Open a console and source your catkin workspace.
2. Change directory to `your_catkin_workspace/src`.
3. Run `git clone https://github.com/iscumd/SharedROSNodes.git`.
4. Run `rosdep install PACKAGE_NAME` for each package:  
`rosdep install isc_joy; rosdep install isc_roboteq_hdc2460; rosdep install isc_roboteq_mdc2460; rosdep install isc_shared_msgs; rosdep install isc_sick`
5. Run `catkin_make` to build the packages.

If you are getting "Permission denied" errors when trying to connect to the Roboteq, add yourself to the `dialout` group with `sudo adduser your_user_name dialout` and log off and on again.
