# SharedROSNodes
ROS nodes we can reuse between project packages.

# Usage
This package targets ROS Kinetic.

1. Open a console and source your catkin workspace.
2. Change directory to `your_catkin_workspace/src`
3. Run `git clone --recurse-submodules https://github.com/iscumd/SharedROSNodes.git`
4. Change directory to the root of your workspace (if you're in `your_catkin_workspace/src`, run `cd ..`).
5. Run `rosdep install --from-paths src --ignore-src -r -y` to install the packages' dependancies.
6. Run `catkin_make` to build the packages.

If you are getting "Permission denied" errors when trying to connect to the Roboteq, add yourself to the `dialout` group with `sudo adduser your_user_name dialout` and log off and on again.
