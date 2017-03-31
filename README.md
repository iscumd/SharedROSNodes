# SharedROSNodes
ROS nodes we can reuse between project packages.

# Usage
This package targets ROS Kinetic.

This package depends on [serial_utils](http://wiki.ros.org/serial_utils), but it has not been listed as compatible with Kinetic yet, so you must clone it into your `/src` from [wjwwood/serial_utils](https://github.com/wjwwood/serial_utils) and it will get compiled with everything else when you `catkin_make`. You should be able to `rosdep install isc_shared` everything else, though you may need to install [serial](http://wiki.ros.org/serial) manually (`sudo apt-get install ros-kinetic-serial`).

If you are getting "Permission denied" errors when trying to connect to the Roboteq, add yourself to the `dialout` group with `sudo adduser your_user_name dialout` and log off and on again.
