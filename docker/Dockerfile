FROM iotbot_base

# using bash
SHELL ["/bin/bash", "-c"]

# remove debconf errors (must run before any apt-get calls)
RUN echo "debconf debconf/frontend select Noninteractive" | debconf-set-selections

# set up iotbot workspace
RUN rm /home/iotbot_ws/build -rf \
    && rm /home/iotbot_ws/install -rf \
    && rm /home/iotbot_ws/src/ -rf \
    && mkdir -p /home/iotbot_ws/src

COPY ./edu_robot /home/iotbot_ws/src/edu_robot
COPY ./edu_robot_control /home/iotbot_ws/src/edu_robot_control

WORKDIR /home/iotbot_ws

# build iotbot_interface first
RUN source /opt/ros/foxy/setup.bash \
    && colcon build --packages-select edu_robot edu_robot_control

# build other nodes after interface to find interface messages in those nodes
RUN source /opt/ros/foxy/setup.bash \
    && source /home/iotbot_ws/install/setup.bash \
    && MAKEFLAGS="-j4 -l1" colcon build --symlink-install --executor sequential

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
    && echo "source /home/iotbot_ws/install/setup.bash" >> ~/.bashrc

CMD source /home/iotbot_ws/install/setup.bash \
    && (ros2 launch edu_robot_control robot_remote_control.launch.py \
    & ros2 run edu_robot iotbot-shield)