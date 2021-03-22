# **dancer_robot**
The control of robot
### ros安装
添加镜像源
```
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```
添加秘钥
```
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
更新
```
sudo apt-get update
```
安装melodic版本(对应ubuntu18)
```
sudo apt-get install ros-melodic-desktop-full
```
初始化rosdep
```
sudo apt install python3-catkin-pkg
sudo apt install python3-rospkg
sudo apt install python3-rosdep-modules
sudo apt install python3-rosdep
sudo rosdep init
```
rosdep更新报错
```
sudo gedit /etc/hosts
```
最后一行添加
```
151.101.84.133  http://raw.githubusercontent.com
185.199.111.133 http://raw.githubusercontent.com
185.199.109.133 http://raw.githubusercontent.com
185.199.110.133 http://raw.githubusercontent.com
185.199.108.133 http://raw.githubusercontent.com
0.0.0.0 http://raw.githubusercontent.com
```
修改list
```
sudo gedit /etc/ros/rosdep/sources.list.d/20-default.list
```
将全部内容
```
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx
# generic
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte
```
修改为
```
# os-specific listings first
#yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx
yaml file:///etc/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx
# generic
#yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
#yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
#yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
#gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte
yaml file:///etc/ros/rosdistro/master/rosdep/base.yaml
yaml file:///etc/ros/rosdistro/master/rosdep/python.yaml
yaml file:///etc/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro file:///etc/ros/rosdistro/master/releases/fuerte.yaml fuerte
```
直接下载list里需要的文件,复制到/etc/ros目录下
```
sudo cp -r /home/zjunlict-vision-1/Desktop/dancer_robot/rosdistro /etc/ros
```
再更新rosdep
```
sudo apt update
rosdep update
```
设置环境变量
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
报错shopt,查看当前shell类型
```
echo $SHELL
```
是zsh,设置环境变量
```
echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```
出错找不到文件setup.bash,随便安装个包
```
sudo apt-get install ros-melodic-turtlesim
gedit ~/.zshrc
source ~/.zshrc
rosdep update
```
报错找不到distribution.yaml,继续修改list
```
sudo gedit /etc/ros/rosdep/sources.list.d/20-default.list
```
在最后一行添加
```
yaml file:///etc/ros/rosdistro/master/kinetic/distribution.yaml
yaml file:///etc/ros/rosdistro/master/melodic/distribution.yaml
yaml file:///etc/ros/rosdistro/master/foxy/distribution.yaml
yaml file:///etc/ros/rosdistro/master/index-v4.yaml
```
又报错找不到fuerte.yaml,修改域名
```
sudo gedit /etc/resolv.conf
```
在nameserver 127.0.0.53之后添加
```
nameserver 8.8.8.8 #google域名服务器
nameserver 8.8.4.4 #google域名服务器
```
还是报错找不到index-v4.yaml
```
???
```
运行
```
rosdep update
```
安装rosinstall
```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
### 环境配置
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

### 运行模型
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
### 效果图片

<img width=850 src="https://img-blog.csdnimg.cn/2020122515203981.png" alt="gazebo仿真-蓝天白云"/>


gazebo仿真-蓝天白云


<img width=850 src="https://img-blog.csdnimg.cn/20201225152328705.png" alt="gazebo仿真-绿茵球场"/>


gazebo仿真-绿茵球场


### 可能报错
```
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-publisher-gui
sudo apt-get install ros-melodic-gazebo-ros-control
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-fake-localization
sudo apt-get install ros-melodic-pr2-controller-manager
killall gzserver
```
### 代码思路

总的思路是步态代码作为一个节点单开，发送内容为舵机值的topic，周期为10ms

步态代码包括climb.cpp, walk.cpp, onefootlanding.cpp, 这些在dancer-motion中

运行步态代码时运行里面的motion_hub.cpp，用来调度所有的动作，目前我们就是爬起接走路

仿真这边代码要修改一下，变成订阅motion的话题得到舵机值作为line_data，后面的计算方法不用变
