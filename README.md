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

# 可能报错
```
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-publisher-gui
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-fake-localization
sudo apt-get install ros-melodic-pr2-controller-manager
killall gzserver
```
# 代码思路

总的思路是步态代码作为一个节点单开，发送内容为舵机值的topic，周期为10ms

步态代码包括climb.cpp, walk.cpp, onefootlanding.cpp, 这些在dancer-motion中

运行步态代码时运行里面的motion_hub.cpp，用来调度所有的动作，目前我们就是爬起接走路

仿真这边代码要修改一下，变成订阅motion的话题得到舵机值作为line_data，后面的计算方法不用变



