ARG from

FROM ${from}

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
RUN apt-get install -y -q

RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc \
                                         vim \
                                         gedit

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
        apt-get install -y ros-noetic-desktop-full \
            ros-noetic-moveit \
            gazebo11 \
            gazebo11-plugin-base \
            ros-noetic-gazebo-dev \
            ros-noetic-gazebo-plugins \
            ros-noetic-gazebo-ros \
            ros-noetic-gazebo-ros-pkgs \
            ros-noetic-ros-control \
            ros-noetic-gazebo-ros-control \
            ros-noetic-geographic-info \
            ros-noetic-teleop-twist-keyboard \
            ros-noetic-joy \
            ros-noetic-effort-controllers \
            ros-noetic-controller-manager \
            ros-noetic-tf2-sensor-msgs \
            ros-noetic-openslam-gmapping \
            ros-noetic-urdf-sim-tutorial \
            libsdl-image1.2-dev \
            libsdl-dev \
            libbullet-extras-dev \
            python3-rosdep \
            python3-rosinstall \
            python3-rosinstall-generator \
            python3-wstool \
            python3-catkin-tools \
            libcanberra-gtk-module \
            libcanberra-gtk3-module \
            libignition-rendering6-dev \
            cmake \
            libeigen3-dev \
            coinor-libipopt-dev \
            lcov \
            graphviz-dev \
            ros-noetic-pid \
            ros-noetic-rosserial-arduino \
            ros-noetic-rosserial \
            ros-noetic-moveit-planners-chomp \
            ros-noetic-catkin python3-catkin-tools \
            ros-noetic-moveit-visual-tools \
            ros-noetic-rosbridge-server \
            ros-noetic-rosparam-shortcuts \
            ros-noetic-ros-industrial-cmake-boilerplate \
            ros-noetic-qt-advanced-docking \
            ros-noetic-ifopt \
            ros-noetic-visp* && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/noetic/setup.bash"  >> ~/.bashrc && \
    echo "source /workspace/devel/setup.bash"  >> ~/.bashrc

RUN sudo apt install ros-noetic-rqt-joint-trajectory-controller
RUN sudo apt-get install -y ros-noetic-industrial-robot-status-interface \
        ros-noetic-scaled-controllers \
        ros-noetic-pass-through-controllers \
        ros-noetic-ur-client-library \
        ros-noetic-velocity-controllers \
        ros-noetic-force-torque-sensor-controller
RUN sudo apt-get install socat 

RUN apt-get install -y python3-pip
RUN pip install pathlib statistics scipy
RUN apt install python3-pip python3-numpy
RUN python3 -m pip install -U pip 
RUN python3 -m pip install --user  tesseract_robotics tesseract_robotics_viewer

RUN sudo apt-get install -y software-properties-common
RUN add-apt-repository ppa:ros-industrial/ppa
RUN apt-get update
RUN apt-get install taskflow

RUN apt install libqt5svg5-dev
RUN apt install -y libignition-common4-dev
# RUN apt install qtbase5-private-dev
# RUN apt-get install -y libqt-advanced-docking-system-dev

RUN apt-get install ros-noetic-joint-state-publisher-gui
RUN apt-get install ros-noetic-franka-description
RUN apt-get install ros-noetic-joint-trajectory-controller

# URTD lib
RUN sudo apt-get install libboost-all-dev

RUN apt-get update && apt-get install -y iputils-ping
RUN apt-get install socat -y

RUN pip install pymodbus
RUN sudo apt-get install ros-noetic-rviz-visual-tools

# for tesseract-qt
RUN sudo apt-get update && apt-get -y install wget lsb-release gnupg
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN sudo apt-get update && apt-get install -y libignition-rendering7-dev
RUN sudo apt install -y libignition-common5-dev
RUN sudo apt install -y python3-roslib python3-sensor-msgs python3-opencv

# for octomap
RUN pip3 install open3d --ignore-installed PyYAML