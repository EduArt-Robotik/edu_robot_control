download-sources:
	current_dir = $(notdir $(shell pwd))
	mkdir -p /tmp/eduart_robot_ws
	cp Dockerfile /tmp/eduart_robot_ws
	cd /tmp/eduart_robot_ws
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot.git
	git clone --branch main https://github.com/EduArt-Robotik/edu_robot_control.git
	cd $(current_dir)

build-docker: download-sources
	current_dir = $(notdir $(shell pwd))
	cd /tmp/eduart_robot_ws
	docker build -t eduard-iotbot .
	cd $(current_dir)

all: download-sources build-docker
	echo "build edu_robot and edu_robot_control into a docker image"
