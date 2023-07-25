FROM eduartrobotik/eduart-robot-control:0.3.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive
ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml

USER root
RUN apt-get update \
    && apt-get install -y \
    python3-pip \
    libhidapi-dev \
    && apt-get clean
USER user
RUN pip3 install hid

COPY ./iot_cyclone_dds_profile.xml /home/$USER/
COPY ./edu_hid_joy /home/$USER/ros/src/edu_hid_joy

# build EduArt packages nodes
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch control node and joy interpreter
CMD echo "launching eduard iot2050 robot control software" \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/user/ros/install/setup.bash \  
    && ros2 launch edu_robot_control robot_remote_control_iot2050.launch.py
