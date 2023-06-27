FROM eduartrobotik/eduart-robot-control:0.2.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# launch control node and joy interpreter
CMD export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/user/iotbot_ws/install/setup.bash \  
    && ros2 launch edu_robot_control robot_remote_control.launch.py
