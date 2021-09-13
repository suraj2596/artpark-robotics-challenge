FROM osrf/ros:melodic-desktop-full

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV CATKIN_WS=/root/cerberus_ws

RUN apt update
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
    geographiclib-tools
    
COPY ./ $CATKIN_WS/src/cerberus/
WORKDIR $CATKIN_WS
RUN catkin config \
      --extend /opt/ros/melodic \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release && \
    catkin build

