FROM eduartrobotik/eduart-robot-control:0.3.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

COPY ./iot_cyclone_dds_profile.xml /home/$USER/

ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml

# launch control node and joy interpreter
CMD echo "launching eduard iot2050 robot control software" \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/user/ros/install/setup.bash \  
    && ros2 launch edu_robot_control robot_remote_control_iot2050.launch.py
