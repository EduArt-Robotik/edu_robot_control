image_name = eduard-robot-control-iotbot:0.2.0

build-docker:
	docker build -t $(image_name) .

build-multiarch-and-push:
	docker buildx build --platform linux/arm64/v8 -t eduartrobotik/$(image_name) --push .

upload-docker:
	docker tag $(image_name) eduartrobotik/$(image_name)
	docker push eduartrobotik/$(image_name)

