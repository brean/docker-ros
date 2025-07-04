# Part 12: Deploy to a robot
On your robot you want to know exactly what software is running. Its often the case that, after testing and deploying to your robot you make changes specifically for your system on it. These changes sometimes do not find their way back into documentation or your code base. So instead lets look at different ways to deploy Docker images to the robot.

### versioning
In your compose.yml you can define an image name, we use this to give our image a tag including a version number and its version for example "jazzy_arm64_001" for a jazzy-based on arm64 with version number 0.0.1.

### cross-building for another architecture
To cross-build for another architecture (if you have an Intel/AMD or M1-mac CPU and your robot has an ARM64, for example a Raspberry Pi) you need to cross-build for this other architecture. This is fairly easy, just create a new builder for the arm64 architecture using buildx, if you define the services inside your compose.yml file they will just build. A script that creates a new builder if needed and runs docker could look like this:
```bash
#!/bin/bash
# Set the name of the builder
BUILDER_NAME="my_builder"

# Check if the builder already exists
if docker buildx inspect "$BUILDER_NAME" > /dev/null 2>&1; then
    echo "Builder '$BUILDER_NAME' already exists."
else
    echo "Builder '$BUILDER_NAME' does not exist. Creating a new one..."
    
    # Create a new builder with the docker-container driver
    docker buildx create --name "$BUILDER_NAME" --use --driver docker-container
    
    # Inspect the newly created builder
    docker buildx inspect --bootstrap "$BUILDER_NAME"
    
    echo "Builder '$BUILDER_NAME' created and set as the active builder."
fi
docker compose build
```

After building you can optionally push directly to your configured docker registry using `docker compose push`. Don't forget to login to your registry by calling `docker login YOUR_REGISTRY` first. `dockerhub` is configured as default, so a simple `docker login` is enough if you want to publish there.

### Without deploying to dockerhub
If you don't want to publish your docker image to dockerhub or your robot does not have access to dockerhub you can simply save the build image and load it on the robot like this:
```bash
# on your PC
docker save -o my_image.tar my_image:jazzy_arm64_001
# copy this to the robot, change X.X.X.X to the actual IP of the robot, do NOT remove the colon (:) at the end
scp my_image.tar robot@X.X.X.X:

# now SSH  on the robot and load the image like this:
docker load -i my_image.tar
```
Because the image can be a few GB big its recommended to connect PC and robot via cable.

### Example project
As an example for a repository that can be used in production to build a docker container you can simply deploy on a raspberry pi take a look at the [Docker Environment for the Kobuki base](https://github.com/helloric/docker-env-kobuki)
