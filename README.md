# wheellegged-gazebo-ros 仿真使用说明

## 1. 环境搭建

- 安装 Ubuntu 20.04 桌面操作系统

- 按此链接安装 ROS：http://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装

- 按以下 shell 命令安装其它依赖：

  ```
  sudo apt-get update
  sudo apt install ros-noetic-urdf \
                 ros-noetic-kdl-parser \
                 ros-noetic-urdf-parser-plugin \
                 ros-noetic-hardware-interface \
                 ros-noetic-controller-manager \
                 ros-noetic-controller-interface \
                 ros-noetic-controller-manager-msgs \
                 ros-noetic-control-msgs \
                 ros-noetic-ros-control \
                 ros-noetic-gazebo-* \
                 ros-noetic-rqt-gui \
                 ros-noetic-rqt-controller-manager \
                 ros-noetic-plotjuggler* \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
                 python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
  ```

## 2、编译

```
mkdir -p ~/limx_ws/src

cd ~/limx_ws/src
git clone https://github.com/limxdynamics/robot-description.git
git clone https://github.com/limxdynamics/wheellegged-gazebo-ros.git

cd ~/limx_ws
catkin_make install
```
