# Dev Container and ROS 2
This repository should give you some examples how to use Visual Studio Code and dev container.

## Installation
You need
1. VS Code (it does not work with Codium)
1. docker (docker-ce)
1. (optional) docker-nvidia

# Part 1: The basics docker and ROS without devcontainer
In this tutorial we look at a ROS 2 system with a custom ros image each, one package that publishes and a second container with a ROS 2 package that has a subscriber.

The application for this could be one ROS-package that you install on your PC while the other is running on your robot.

Open a terminal, cd into the "part1"-folder and type `docker compose up`. It will build and run the 2 docker images and start 2 docker container, one for publishing and one for subscribing, one outputs to the local docker network and the other is receiving these messages.

This is nice as you now have a ROS 2 setup system-independent 
All files are build in your container, you need to rebuild the docker container with `docker compose build` every time you change any file in your ros-packages.

# Part 2: Basic ROS with docker
For development purposes we have a third docker image where we put both packages in one image for the dev-container.

# Part 3: Camera stream with RVIZ and C++
TODO: rviz and ros2 bag
TODO: recompile?

# Part 4: Include a webserver to control a robot
TODO: include https://github.com/brean/svelte-robot-control

# Part 5: Deploy to a robot
TODO: build package, push to robot (with and without publishing to github)