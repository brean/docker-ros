# Dev Container and ROS 2
This repository should teach you how to work with docker and ros in a modern development setup using Visual Studio Code and dev container.

## Installation
You need
1. [VS Code](https://code.visualstudio.com/) (devContainer do not work with Codium)
1. docker ([docker-ce](https://docs.docker.com/engine/install/) recommended)
   1. don't forget to do the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) (creating the `docker`-group and add yourself)
1. (optional) [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) if you have an nvidia-gpu and don't want software rendering for 3D tools like [rviz](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html).

# Part 1: The basics docker and ROS without devcontainer
First lets take a look at a simple ROS 2 setup with two custom ros packages in the `ros`-folder, it contains one package that publishes and a second package with a ROS 2 package that has a subscriber to these messages. Its taken from the [tutorial for writing a basic publisher and subscriber in python.](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) So it should be easy to understand however both single subscriber and publisher scripts are accompanied by a some files to create individual packages for both as you might find in a more complex ROS 2 setup.

The application for this could be one ROS-package that you install on your PC while the other is running on your robot (often times accompanied by a third package that contains shared ROS-messages on both but as we are only sending basic strings we use the `std_msgs`-package that comes with our ROS 2 installation).

Open a terminal, cd into the `part1`-folder of this repository and type `docker compose up publishing subscribing`. It will build and run the 2 docker images and start 2 docker container, one for publishing and one for subscribing, one outputs to the local docker network and the other is receiving these messages.
The source for both packages can be found in the `ros/`-folder

When you press CTRL+C both container should exit, if you just close the terminal window don't forget to run `docker compose down` to make sure both docker container are stopped.

This is nice as you now have a ROS 2 setup system-independent 
All files are build in your container, however you need to rebuild the docker container with `docker compose build` every time you change any file in your ros-packages and the files from the docker image differ from your local files. 

**Task 1**: 
1. Build the container using `docker compose build` in the `part1`-folder
1. Change code in the `ros`-folder, for example change line 18 in the file `ros/publishing/publishing/main.py` from `msg.data = 'Hello World: %d' % self.i` to `msg.data = 'Sending: %d' % self.i`
1. restart the docker container without building by just running `docker compose up` in the `part1`-folder. You will still see the `Hello World`-messages.
1. re-run the `docker compose build`-command and then restart with `docker compose up`, now you should see the updated data getting published.

# Part 2: Docker Volumes
Using docker volumes you can increase your development speed: you don't have to re-run the `docker compose build` command all the time when you change code. Note that this only helps you for interpreted languages like python, for C/C++ applications it makes more sense to rebuild the whole package. 

**Task 2**:
1. change into the `part2`-folder and run `docker compose build`
1. run `docker compose up` and keep it running
1. change the code, for example instead of "I heard: " you could print out something like "I received: " (in `ros/subscribing/subscribing/main.py`)
1. save the file. You'll notice that the output of your docker compose command still continues with "I heared:"
1. stop both docker container with CTRL+C and directly recreate it with `docker compose up`.
1. Now you see the "I received:" - without the need of rebuilding the docker container!

Note that we build the packages with `colcon` as part of the `docker compose build`-process. This means that only the code at the time you build the image gets copied into your docker image. If you remove the volume your old code will be used again, as it is stored in the image. This also means that when you have bigger changes or add new packages you want to rebuild the image.

Volumes help with smaller changes while developing your code but you still need to rebuild the image when dependencies change or when you want to add additional nodes.

But this already makes it nicer to work with docker and ROS writing python, but we still have to restart the docker container manually, it would be nice if we could watch the file system for changes and restart our ros-node automatically, so lets do that next!

# Part 3: (optional) automatically reloading using watchdog
For this we need to deviate from the examples and extend our code. 

In this part we extend on the previous tutorial and use the functionality of 

TODO: https://python-watchdog.readthedocs.io/en/stable/api.html

In the next part we will take a look at devContainer so we can run commands directly from inside the VS Code terminal inside the docker container to increase our productivity even more.

# Part 4: Basic ROS with docker in a devContainer
For development purposes we have a third docker image where we put both packages in one image for the dev-container.

# Part 3: Camera stream with RVIZ and C++
TODO: rviz and ros2 bag
TODO: recompile?

# Part 4: Include a webserver to control a robot
TODO: include https://github.com/brean/svelte-robot-control

# Part 5: Deploy to a robot
TODO: build package, push to robot (with and without publishing to github)

