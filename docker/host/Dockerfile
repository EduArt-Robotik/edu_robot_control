FROM ubuntu:jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV UBUNTU_DISTRO jammy
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
    apt-utils \
    software-properties-common \
    nano \
    curl \
    gnupg \
    git \
    build-essential \
    sudo \
########################################################
# Install ROS
## Get package archive key and ROS source list entry.
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_DISTRO main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
########################################################
# Install ROS packages
    && apt-get install -q -y --no-install-recommends \
    ros-$ROS_DISTRO-ros-base \
    python3-colcon-common-extensions \
    software-properties-common \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# write source commands to .bashrc -> no need to source afterwards
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc \
########################################################
# Add non root user for development
    && useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USER/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml" >> /home/$USER/.bashrc \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/eduart_ws/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

COPY ./edu_robot /home/$USER/eduart_ws/src/edu_robot
COPY ./edu_robot_control /home/$USER/eduart_ws/src/edu_robot_control
COPY ./joystick_drivers /home/$USER/eduart_ws/src/joystick_drivers
COPY ./fastrtps_profile.xml /home/$USER

WORKDIR /home/$USER/eduart_ws

RUN export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

CMD source /home/user/eduart_ws/install/setup.bash \
    && ros2 launch edu_robot_control eduard_monitor.launch.py
