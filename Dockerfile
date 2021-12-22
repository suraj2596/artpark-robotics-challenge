FROM osrf/ros:melodic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV CATKIN_WS=/root/cerberus_ws

RUN apt-get update && \
    apt-get install -y software-properties-common && \
    rm -rf /var/lib/apt/lists/*
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

RUN apt install -y \
    vim \
    tmux \
    htop \
    python-catkin-tools \
    ros-melodic-navigation \
    ros-melodic-rtabmap-ros \
    ros-melodic-turtlebot3-simulations \
    ros-melodic-turtlebot3-navigation \
    ros-melodic-dwa-local-planner \
    ros-melodic-move-base \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-dwa-local-planner* \
    ros-melodic-geographic-msgs \
    libgeographic-dev \
    geographiclib-tools \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg
    
COPY ./ $CATKIN_WS/src/cerberus/
WORKDIR $CATKIN_WS
RUN catkin config \
      --extend /opt/ros/melodic \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build

