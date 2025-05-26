FROM curobo_docker:x86

# Add camera azure kinect
RUN apt update && apt install software-properties-common \
                    ros-humble-librealsense2* \
                    ros-humble-realsense2-* -y &&\
                    apt-add-repository -y -n 'deb http://archive.ubuntu.com/ubuntu focal main' && \
                    apt-add-repository -y 'deb http://archive.ubuntu.com/ubuntu focal universe'
RUN apt-get install -y libsoundio1
RUN apt-add-repository -r -y -n 'deb http://archive.ubuntu.com/ubuntu focal universe' && \
    apt-add-repository -r -y 'deb http://archive.ubuntu.com/ubuntu focal main'


RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb > /tmp/libk4a1.3_1.3.0_amd64.deb
RUN echo 'libk4a1.3 libk4a1.3/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections
RUN sudo dpkg -i /tmp/libk4a1.3_1.3.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb > /tmp/libk4a1.3-dev_1.3.0_amd64.deb
RUN sudo dpkg -i /tmp/libk4a1.3-dev_1.3.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb > /tmp/libk4abt1.0_1.0.0_amd64.deb
RUN echo 'libk4abt1.0	libk4abt1.0/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections
RUN sudo dpkg -i /tmp/libk4abt1.0_1.0.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb > /tmp/libk4abt1.0-dev_1.0.0_amd64.deb
RUN sudo dpkg -i /tmp/libk4abt1.0-dev_1.0.0_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb > /tmp/k4a-tools_1.3.0_amd64.deb
RUN sudo dpkg -i /tmp/k4a-tools_1.3.0_amd64.deb

COPY 99-k4a.rules /etc/udev/rules.d/99-k4a.rules

# Add realsense lib
# RUN apt install 

# Add azure, doosan, tool_box and pcd_fuse ros2 package
WORKDIR /home/ros2_ws/src

# Add point cloud fusion
RUN git clone -b humble-devel https://github.com/doosan-robotics/doosan-robot2.git && \
    git clone -b humble https://github.com/ros-controls/gz_ros2_control && \
    git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git 

RUN sed -i '771d' /home/ros2_ws/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp

### install gazebo sim for doosan package
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update
RUN apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs\
        ros-humble-ros-gz-sim ros-humble-ros-gz libpoco-dev libyaml-cpp-dev\
        ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro\
        ros-humble-joint-state-publisher-gui ros-humble-ros2-control\
        ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs\
        dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group libignition-gazebo6-dev


WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"
