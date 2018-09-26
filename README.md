# SharedROSNodes
ROS nodes we can reuse between project packages.

# Usage
This package targets ROS Kinetic.

1. Download the repository.
2. Run `rosdep install PACKAGE_NAME` for each package.  
`rosdep install isc_joy; rosdep install isc_roboteq_hdc2460; rosdep install isc_roboteq_mdc2460; rosdep install isc_shared_msgs; rosdep install isc_sick`
3. Run `catkin_make` to build the packages.

If you are getting "Permission denied" errors when trying to connect to the Roboteq, add yourself to the `dialout` group with `sudo adduser your_user_name dialout` and log off and on again.
