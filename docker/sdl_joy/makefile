image_name = eduard-robot-control-sdl-joy:0.4.0

download-sources:
	git clone --branch feature/reduce_complexity_for_stable_processing_dt https://github.com/EduArt-Robotik/joystick_drivers.git

build-docker:
	docker build -t $(image_name) .

build-multiarch-and-push:
	docker buildx build --platform linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)
