FROM eduartrobotik/eduart-robot:0.5.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

USER root
RUN apt-get update \
    && apt-get install -y \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-xacro


# work with user user
USER $USER

# set up  workspace
COPY ./edu_robot_control /home/$USER/ros/src/edu_robot_control

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
