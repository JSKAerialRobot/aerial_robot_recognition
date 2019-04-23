# aerial_robot_recoginition
Recognition methods about aerial robot

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -u -t src aerial_robot_recognition http://github.com/tongtybj/aerial_robot_recognition --git
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```