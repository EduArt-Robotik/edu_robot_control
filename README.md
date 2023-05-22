# edu_robot_control
Provides nodes, robot description for RViz and other tools for remote control of Eduart robots.

docker run --user user --name eduard-robot-control-0.2.0 --restart=always --privileged -v /dev:/dev --net=host --pid=host --ipc=host --env EDU_ROBOT_NAMESPACE=eduard/red eduard-robot-control:0.2.0