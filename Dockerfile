FROM dorowu/ubuntu-desktop-lxde-vnc:bionic

RUN apt-get update 
RUN echo "y"| apt-get install vim && echo "y"| apt-get install gedit && echo "y"| apt-get install git

RUN apt-get clean && apt-get update && echo "y"| apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN apt update && echo "y"| apt install curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt update && echo "y"| apt install ros-dashing-desktop

RUN echo "y"| apt install python3-argcomplete

#Install additional RMW implementations
RUN apt update && echo "y"| apt install ros-dashing-rmw-opensplice-cpp 
#&& echo "y"| apt install ros-dashing-rmw-connext-cpp # for RTI Connext (requires license agreement)

#install ros1
ENV ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
RUN apt-get update
RUN echo "y" | apt-get install gpg-agent
RUN apt-get install dirmngr

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN echo "y" | apt-get install ros-melodic-desktop
RUN rosdep init && rosdep update
RUN /bin/bash -c "echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc"


#Dependencies for building packages
RUN echo "y"| apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

#Install additional packages using ROS 1 packages
RUN apt update && echo "y"| apt install ros-dashing-ros1-bridge

#Install gazebo

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN apt-get install wget
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && echo "y"| apt-get install gazebo9 && echo "y"| apt-get install libgazebo9-dev

#install ros2 gazebo package
RUN apt-get update
RUN echo "y"| apt install ros-dashing-gazebo-ros-pkgs

#add ro2 path
RUN /bin/bash -c "echo 'source /opt/ros/dashing/setup.bash' >> /root/.bashrc"

#install colcon
RUN echo "y"| apt-get install python3-colcon-common-extensions


#install rqt
RUN echo "y"| apt install ros-dashing-rqt*

#install moveit
RUN apt-get update
RUN echo "y"| apt-get dist-upgrade
RUN echo "y"| apt-get install ros-melodic-catkin python-catkin-tools
RUN echo "y"| apt-get install ros-melodic-moveit
RUN echo "y"| apt-get install ros-melodic-moveit-visual-tools

#install techman robot ros2
RUN cd /root && mkdir -p tm_ros2_ws/src
ADD robotiq_2f_140_gripper_visualization  /root/tm_ros2_ws/src/robotiq_2f_140_gripper_visualization
ADD techman_robot_utility_scripts  /root/tm_ros2_ws/src/techman_robot_utility_scripts
ADD tm_driver  /root/tm_ros2_ws/src/tm_driver
ADD tm_gazebo_plugin  /root/tm_ros2_ws/src/tm_gazebo_plugin
ADD tm_grasp_description  /root/tm_ros2_ws/src/tm_grasp_description
ADD tm_launch  /root/tm_ros2_ws/src/tm_launch
ADD tm_msgs  /root/tm_ros2_ws/src/tm_msgs
ADD tm_status  /root/tm_ros2_ws/src/tm_status
