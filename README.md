# **dancer_robot**
The control of robot
# 环境配置
```
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
```

```
roslaunch dancer_urdf_model display.launch
```


将dancer_urdf_model中worlds中的car_world.jpg复制进/usr/share/gazebo-9/media/material/textures中，


将dancer_urdf_model中worlds中的gazebo.material复制进/usr/share/gazebo-9/media/material/scripts中。


```
roslaunch dancer_urdf_model gazebo.launch
```

键盘控制：

        'q'         # body_head

        'w'         # body_head2

        'a'         # arm_left

        's'         # hand_left

        'z'         # arm_right

        'x'         # hand_right


        'e'         # body_hip_left

        'r'         # body_hip2_left

        'd'         # leg_left

        'f'         # leg2_left

        'c'         # leg3_left

        'v'         # leg4_left


        't'         # body_hip_right

        'y'         # body_hip2_right

        'g'         # leg_right

        'h'         # leg2_right

        'b'         # leg3_right

        'n'         # leg4_right

