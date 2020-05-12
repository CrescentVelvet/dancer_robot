# **dancer_robot**
The control of robot
# 环境配置
```
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
```
在.bashrc中添加source /home/zjunlict/catkin_ws/devel/setup.bash

将dancer_urdf_model/worlds中的car_world.jpg复制进textures中，
```
sudo cp /home/zjunlict/catkin_ws/src/dancer_robot/dancer_urdf_model/worlds/dancer_world.jpg /usr/share/gazebo-9/media/materials/textures
```
将dancer_urdf_model/worlds中的gazebo.material复制进scripts中。
```
sudo cp /home/zjunlict/catkin_ws/src/dancer_robot/dancer_urdf_model/worlds/dancer_world.material /usr/share/gazebo-9/media/materials/scripts
```
报错[Err] [REST.cc:205] Error in REST request
```
sudo gedit ~/.ignition/fuel/config.yaml
```
将 url : https://api.ignitionfuel.org 注释掉

添加 url: https://api.ignitionrobotics.org

修改climb参数
```
sudo gedit /home/zjunlict/catkin_ws/src/dancer_robot/dancer-motion/config/parameters/motion_hub_param.yaml
```
修改文件中路径为自己的路径

# 运行模型
RVIZ查看模型
```
roslaunch dancer_urdf_model display.launch
```

gazebo进行仿真
```
roslaunch dancer_urdf_model gazebo.launch
```

motion运动控制
```
roslaunch dmotion main.launch
```

键盘控制：

​       'q' 	# body_head
​       'w'	# body_head2
​       'a'	# arm_left
​       's'	# hand_left
​       'z'	# arm_right
​       'x'	# hand_right
​       'e'	# body_hip_left
​       'r'	# body_hip2_left
​       'd'	# leg_left
​       'f'	# leg2_left
​       'c'	# leg3_left
​       'v'	# leg4_left
​       't'	# body_hip_right
​       'y'	# body_hip2_right
​       'g'	# leg_right
​       'h'	# leg2_right
​       'b'	# leg3_right
​       'n'	# leg4_right

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



