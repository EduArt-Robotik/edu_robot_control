image_name = eduart-robot-control:0.5.0

download-sources:
	git clone --branch 0.5.0 https://github.com/EduArt-Robotik/edu_robot_control.git
	git clone --branch feature/reduce_complexity_for_stable_processing_dt https://github.com/EduArt-Robotik/joystick_drivers.git

build-docker:
	docker build -t $(image_name) .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

build-multiarch-and-push:
	docker buildx build --platform linux/amd64,linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

all: download-sources build-docker
	$(info Creating EduArt's robot control library Docker image "$(image_name)".)

clean:
	rm -rf edu_robot_control
