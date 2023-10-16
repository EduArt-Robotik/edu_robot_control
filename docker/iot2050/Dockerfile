FROM eduartrobotik/eduart-robot-control:0.3.1

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# remove joy node provided by ROS
USER root
RUN apt-get remove -y ros-$ROS_DISTRO-joy

# work with user user
USER $USER

# install custom joy node
COPY ./joystick_drivers home/$USER/ros/src/joystick_drivers
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --packages-select joy --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# in case cyclone is used this profile is needed
COPY ./iot_cyclone_dds_profile.xml /home/$USER/
ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN sudo chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# launch control node and joy interpreter
CMD [ "ros2", "launch", "edu_robot_control", "robot_remote_control_iot2050.launch.py" ]
