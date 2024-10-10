FROM eduartrobotik/eduart-robot-control:0.4.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO jazzy
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# installing of linux joy node
USER root
RUN apt-get update \
    && apt-get install -y ros-$ROS_DISTRO-joy-linux

USER $USER

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# launch control node and joy interpreter
CMD [ "ros2", "launch", "edu_robot_control", "robot_remote_control_ipc127e.launch.py" ]
