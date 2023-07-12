FROM eduartrobotik/eduart-robot:0.3.0

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# work with user user
USER $USER

# set up  workspace
COPY ./edu_robot_control /home/$USER/iotbot_ws/src/edu_robot_control

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
