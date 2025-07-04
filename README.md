# Docker and ROS 2 for developers
This repository should teach you how to work with the [docker engine](https://docs.docker.com/engine/) and the latest stable ROS 2 LTE release [jazzy](https://docs.ros.org/en/jazzy/) in a modern development setup using [Visual Studio Code](https://code.visualstudio.com/) and [Dev Container](https://code.visualstudio.com/docs/devcontainers/create-dev-container). It also provides a source of example files and links to examples and projects that you can use to build more complex robot systems.

(_Not all of these are done, I just manged to finish the parts with the checkmark ✅_).

If you need knowledge from any of the unfinished chapter feel free to contact me, I will then prioritze that chapter. If you like to help with documentation also feel free to contact me.

 - ✅ [Part 1](01_docker_basics.md) introduces a basic ROS 2 and Docker setup with 2 Docker container connected over the internal Docker setup using **docker compose**.
 - ✅ [Part 2](02_docker_volumes.md) teaches basics about **Docker volumes**
 - ✅ [Part 3](03_more_volumes.md) works with dynamically **changing files** and docker volumes.
 - ❌ [Part 4](04_dev_container.md) [issue](https://github.com/brean/docker-ros/issues/2) introduces **Dev Container** and using **breakpoints** in VS Code
 - ❌ [Part 5](https://github.com/brean/docker-ros/issues/6) dives into best-practices by creating Unit tests for component and integration **Testing**.
 - ❌ [Part 6](https://github.com/brean/docker-ros/issues/4) includes **visual ros-tools** (rviz2) using Wayland
 - ❌ [Part 7](https://github.com/brean/docker-ros/issues/1) connects to real devices from inside docker, revisiting volumes and the special **device** option to use USB-Devices like Gamepads, lidar-scanner or the kobuki base.
 - ✅ [Part 8](08_microros_development.md) is a small detour to a more custom c-code for Raspberry Pi Pico2
 - ❌ [Part 9](https://github.com/brean/docker-ros/issues/7) streams images from a WebCam in a more complex remote-control setup
 - ❌ [Part 10](https://github.com/brean/docker-ros/issues/8) extends part 9 to not only stream camera images but also connection to other systems, how docker systems can be used in a "cloud robotics" **server setup**.
 - ❌ [Part 11](https://github.com/brean/docker-ros/issues/9) discusses dependency management, we evaluate different systems that can be used to manage dependencies, from just using the apt **package manager** to more robotics specific systems like **vcstool** or **autoproj**.
 - ✅ [Part 12](12_deploy_docker_image.md) gives an example on how to deploy the docker image to your robot, how you cross-build using **dockerx** for your robots CPU and some best practices (From the [HelloRIC](https://github.com/helloric)-project).
 - ❌ [Part 13](13_ros_network.md) [issue](https://github.com/brean/docker-ros/issues/10) is about docker networking including possible pitfalls and issues with the **Docker network with ROS2**.
 - ❌ [Part 14](https://github.com/brean/docker-ros/issues/11) is a bit specific for development at [DFKI-RIC](https://robotik.dfki-bremen.de/en/startpage), we take a look at the Open Source **Docker Image Development** environment. Even if you are not connected to DFKI, maybe it also makes sense for you to use or modify it to streamline your development process.

This tutorial is directed towards ROS 2 (mostly python) developers who want to accelerate their development process, ROS 2 basics are assumed.

## TL:DR!
You don't want to read it all but need to get your robot working quickly?

Here are some exisiting solutions for some problems you might have:

 - You want to use a python debugger (debugpy) take a look at [Part 4](04_dev_container.md)
 - You need to send images with limited bandwidth over network, use the [WebRTC-Bridge](https://github.com/brean/webrtc_bridge). (More in Part 9)
 - For a MicroROS-example see [MicroROS firmeare for the Huginn robot](https://github.com/brean/microros_firmware_huginn) as example project with PWM/PPM control for different motors (More in [Part 8](08_microros_development.md))
 - Copy a docker image to a robot see [Part 12](12_deploy_docker_image.md)
 - You have a network issue and like to follow a checklist what to do? see [Part 13](13_ros_network.md) (WIP)

## Installation
For all parts you need
1. [VS Code](https://code.visualstudio.com/download) (If you want to use [Codium](https://vscodium.com/) you can use [DevPodContainers](https://github.com/3timeslazy/vscodium-devpodcontainers), however it requires some more steps to setup and is still experimental so we focus on VS Code).
1. docker ([docker-ce](https://docs.docker.com/engine/install/) recommended)
   1. don't forget to do the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) (creating the `docker`-group and add yourself)
1. The [VS Code Dev Container extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) (not needed for the first 3 parts)
1. (optional) [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) if you have an nvidia-gpu and don't want software rendering for 3D tools like [rviz](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html), starting at step 5.
1. Beginner to intermediate ROS 2 and python knowledge, some docker basics. The [Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) on the ROS 2 website are a good start be aware of a steep learning curve if you are new to ROS!
1. Some disk space as docker container can take up some GB.
