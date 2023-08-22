FROM eduartrobotik/eduart-robot-control:0.3.1

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# installing of linux joy node
USER root
RUN apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-joy-linux

USER $USER

COPY ./ipc_cyclone_dds_profile.xml /home/$USER/
COPY ./ie_app_cyclone_dds_profile.xml /home/$USER/

ENV CYCLONEDDS_URI=/home/$USER/ipc_cyclone_dds_profile.xml

# launch control node and joy interpreter
CMD echo "launching eduard ipc127e robot control software" \
    && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    # && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/ros/fastrtps_profile.xml \
    && source /home/user/.bashrc \
    && source /home/user/ros/install/setup.bash \
    && ros2 launch edu_robot_control robot_remote_control_ipc127e.launch.py
