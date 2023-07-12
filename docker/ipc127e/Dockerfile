FROM eduartrobotik/eduart-robot-control:0.3.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/ipcbot_ws/fastrtps_profile.xml \
    && source /home/user/ipcbot_ws/install/setup.bash \
    && ros2 launch edu_robot_control robot_remote_control.launch.py
