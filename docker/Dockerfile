FROM ros:galactic-ros-base-focal

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO galactic

RUN \
    apt-get update \
    && apt-get install -y software-properties-common

# install mraa lib for UART communication in shield node
RUN add-apt-repository ppa:mraa/mraa \
    && apt-get update \
    && apt-get install -y libmraa2 libmraa-dev libmraa-java python-mraa python3-mraa mraa-tools

# install Joystick Driver for ROS2 foxy
RUN sudo apt-get install -y ros-$ROS_DISTRO-joy


# set up iotbot workspace
RUN mkdir -p /home/iotbot_ws/src/iotbot

COPY ./edu_robot /home/iotbot_ws/src/edu_robot
COPY ./edu_robot_control /home/iotbot_ws/src/edu_robot_control

WORKDIR /home/iotbot_ws

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /home/iotbot_ws/install/setup.bash" >> ~/.bashrc

CMD source /home/iotbot_ws/install/setup.bash \
    && (ros2 launch edu_robot_control robot_remote_control.launch.py \
    & ros2 launch edu_robot eduard-iot2050.launch.py)
