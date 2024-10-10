# ROS2_turtle_game
本说明包含了安装ubuntu22.04.5(LTS)以及ROS2Humble的运行环境以及Robomaster招新第一题的代码说明
## 环境搭建
### Ubuntu22.04安装
Ubuntu为常见的基于Linux内核的操作系统，或者说发行版本，ROS2对于Linux的支持较为完善，一般有以下几种安装方式
1. 虚拟机
2. 双系统
3. windows子系统
4. Linux to go（移动端linux）
由于本人虚拟机运行因未知原因不太稳定，故而购入硬盘制作Linux to go

### Linux to go 教程

基于https://blog.csdn.net/m0_64545111/article/details/136131918制作
注意：ROS2Humble仅支持Ubuntu22.04,若不慎安装了Ubuntu24.04则需重装系统

### ROS2 Humble安装
1. Ubuntu镜像换源
Ubuntu安装后的官方源默认是美国的服务器，在国内安装软件会受到比较大的限制，下载软件很慢，需要切换成国内源。
方法：
打开系统设置，在软件和更新中更改下载服务器，选择国内源

3. 一键安装ROS2（鱼香ROS开发的一键安装脚本，可通过该脚本安装其他常用软件）
2.1. ctrl+alt+t打开终端

2.2. 输入命令
```
wget http://fishros.com/install -O fishros && . fishros
```

选择ROS2，Humble桌面版完成安装过程。
安装完成后在终端输入ros2检查是否安装成功。

3. 配置ROS2环境
[官方配置文档](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)

3.1 将ros2加入系统默认的环境中
输入以下命令，将ros2加入bashrc中，这样每次启动终端时都可以自动加载ros2

```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

至此，ROS2 Humble已经成功在Ubuntu上安装完毕。可以在终端通过ros2命令来运行ROS2

## 代码运行说明
1.创建ROS2工作空间
在终端中输入以下命令
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 安装代码功能包
在同一目录下输入以下命令：
```
git clone https://github.com/Jackuseful-Citrus/ROS2_turtle_game
```

3. 安装依赖
输入以下命令
```
sudo rosdep init
rosdep update
rosdep install -i --from-path ~/ros2_ws/src --rosdistro humble -y
```

4. 编译工作空间
```
cd ~/ros2_ws
colcon build
```

5. 设置环境
同一目录下输入
```
source install/setup.bash
```

6. 运行ROS2 turtlesim节点
```
ros2 run turtlesim turtlesim_node
```
7. 运行题目代码
开启一个新的终端，输入以下命令
```
source install/setup.bash
ros2 run turtle_controller attack
```
