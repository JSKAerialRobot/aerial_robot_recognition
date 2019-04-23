## Introduction
Augmentation of [jsk_perception](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/jsk_perception) which is specialized for aerial robot

## Samples:
- Detection of the object with single color on the ground. Need the tf of aerial robot w.r.t world frame.
```
$ roslaunch aerial_robot_perception test-single_color_ground_object_detection.test gui:=true
```