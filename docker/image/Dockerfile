FROM eduartrobotik/eduart-robot:0.5.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

USER root
RUN apt-get update \
    && apt-get install -y \
      ros-$ROS_DISTRO-joy-linux

# work with user user
USER $USER

# set up  workspace
COPY ./edu_robot_control /home/$USER/ros/src/edu_robot_control
COPY ./joystick_drivers home/$USER/ros/src/joystick_drivers

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --packages-select joy edu_robot_control --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
