Gazebo Groovy Setup Instructions
=====
* Install Ubuntu 12.04

* Check your video card manufacturer. The following command will list your graphics card adapters.

```
lspci | grep VGA
```

* If you have a single NVIDIA card, follow the directions below. If you have both a NVIDIA and an Intel card listed (i.e. NVIDIA Optimus laptops), follow the tutorial online to install Bumblebee (https://wiki.ubuntu.com/Bumblebee). You may need to manually update the PCI Bus ID in /etc/bumblebee/xorg.conf.nvidia. If you have an ATI/AMD card, the model will not work properly.

* Start with the following command only if you have an NVIDIA graphic card (Which is required to run the model):

```
sudo apt-get update
sudo apt-get install linux-headers-generic
sudo apt-get install nvidia-current
sudo nvidia-xconfig 
```

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

* Restart your computer. 
====
* groovy install

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-groovy-desktop-full
```

* Be sure to hit "no" when it asks if you would like the daemon to start on bootup.  Then continue with the following commands:

```
sudo rosdep init
rosdep update
source /opt/ros/groovy/setup.bash
sudo apt-get install python-rosinstall
rosws init ~/groovy_workspace /opt/ros/groovy
```

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu precise main" > /etc/apt/sources.list.d/drc-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
wget http://packages.osrfoundation.org/drc.key -O - | sudo apt-key add -
```

```
sudo apt-get update
```

```
sudo apt-get install -y cmake debhelper libfreeimage-dev libprotoc-dev libprotobuf-dev protobuf-compiler freeglut3-dev libcurl4-openssl-dev libtinyxml-dev libtar-dev libtbb-dev libogre-dev libxml2-dev pkg-config libqt4-dev ros-groovy-urdfdom ros-groovy-console-bridge libltdl-dev libboost-thread-dev libboost-signals-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libboost-regex-dev libboost-iostreams-dev robot-player-dev libcegui-mk2-dev libavformat-dev libavcodec-dev libswscale-dev libbullet-dev
sudo apt-get install -y cmake debhelper ros-groovy-ros ros-groovy-ros-comm
sudo apt-get install -y cmake debhelper ros-groovy-xacro ros-groovy-ros libboost-dev ros-groovy-image-common ros-groovy-ros-comm ros-groovy-common-msgs avr-libc gcc-avr libqt4-dev
sudo apt-get install -y cmake debhelper ros-groovy-pr2-mechanism ros-groovy-std-msgs ros-groovy-common-msgs ros-groovy-image-common ros-groovy-geometry ros-groovy-pr2-controllers ros-groovy-geometry-experimental ros-groovy-robot-model-visualization ros-groovy-image-pipeline ros-groovy-image-transport-plugins
```

```
sudo apt-get install ros-groovy-control-msgs ros-groovy-pr2-controllers ros-groovy-diagnostics-monitors ros-groovy-gscam 
```
======
* gazebo install

```
sudo apt-get update
sudo apt-get install gazebo osrf-common sandia-hand
sudo apt-get install drcsim
```

```
sudo apt-get install ros-groovy-moveit-full 
```

```
echo source ~/groovy_workspace/setup.bash >> ~/.bashrc
echo source /usr/share/drcsim/setup.sh >> ~/.bashrc
echo export ROS_WORKSPACE=\~/groovy_workspace/ >> ~/.bashrc
echo export ROS_PACKAGE_PATH=\$ROS_WORKSPACE:\$ROS_PACKAGE_PATH >> ~/.bashrc
. ~/.bashrc
```
