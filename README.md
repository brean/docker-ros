# Dev Container and ROS 2
This repository should teach you how to work with docker and ros in a modern development setup using Visual Studio Code and dev container.

## Installation
You need
1. [VS Code](https://code.visualstudio.com/) (If you want to use [Codium](https://vscodium.com/) you can use DevPod, however it requires some more steps to setup and is a bit harder to debug).
1. docker ([docker-ce](https://docs.docker.com/engine/install/) recommended)
   1. don't forget to do the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) (creating the `docker`-group and add yourself)
1. (optional) [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) if you have an nvidia-gpu and don't want software rendering for 3D tools like [rviz](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html).

# Part 1: The basics docker and ROS without devcontainer
Even if you want to use devContainer it is best practice to have a setup that also runs without it so people can run your container even if something does not work with their setup or if they want to use another ide then VS Code. I prefere `docker compose` to configure my docker container so I don't have to run long `docker`-commands.

First lets take a look at a simple ROS 2 setup with two custom ros packages in the `ros`-folder, it contains one package that publishes and a second package that has a subscriber to these messages. Its taken from the [tutorial for writing a basic publisher and subscriber in python.](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) So it should be easy to understand however both single subscriber and publisher scripts are accompanied by a some files to create individual packages and launch files for both as you might find in a more complex ROS 2 setup.

The application for this could be one ROS-package that you install on your PC while the other is running on your robot (often times accompanied by a third package that contains shared ROS-messages on both but as we are only sending basic strings we use the `std_msgs`-package that comes with our ROS 2 installation).
We will look at more realistic use cases in the later parts.

Open a terminal, cd into the `part1`-folder of this repository and type `docker compose up publishing subscribing`. It will build and run the 2 docker images and start 2 docker container, one for publishing and one for subscribing, one outputs to the local docker network and the other is receiving these messages.
The source for both packages can be found in the `ros/`-folder

When you press CTRL+C both container should exit, if you just close the terminal window don't forget to run `docker compose down` to make sure both docker container and the network are removed.

This is nice as you now have a ROS 2 setup system-independent 
All files are build in your container, however you need to rebuild the docker container with `docker compose build` every time you change any file in your ros-packages and the files from the docker image differ from your local files. 

**Try for yourself**: 
1. Build the container using `docker compose build` in the `part1`-folder
1. Change code in the `ros`-folder, for example change line 18 in the file `ros/publishing/publishing/main.py` from `msg.data = 'Hello World: %d' % self.i` to `msg.data = 'Sending: %d' % self.i`
1. restart the docker container without building by just running `docker compose up` in the `part1`-folder. You will still see the `Hello World`-messages.
1. re-run the `docker compose build`-command and then restart with `docker compose up`, now you should see the updated data getting published.

# Part 2: Docker Volumes
Using docker volumes you can increase your development speed: you don't have to re-run the `docker compose build` command all the time when you change code. Note that this only helps you for interpreted languages like python, for C/C++ applications it makes more sense to rebuild the whole package. 

**Try for yourself**:
1. change into the `part2`-folder and run `docker compose build`
1. run `docker compose up` and keep it running
1. change the code, for example instead of "I heard: " you could print out something like "I received: " (in `ros/subscribing/subscribing/main.py`)
1. save the file. You'll notice that the output of your docker compose command still continues with "I heared:"
1. stop both docker container with CTRL+C and directly recreate it with `docker compose up`.
1. Now you see the "I received:" - without the need of rebuilding the docker container!

Note that we build the packages with `colcon` as part of the `docker compose build`-process in the Dockerfile. This means that only the code at the time you build the image gets copied into your docker image. If you remove the volume your old code will be used again, as it is stored in the image. This also means that when you have bigger changes or add new packages you want to rebuild the image.

Volumes help with smaller changes while developing your code but you still need to rebuild the image when dependencies change or when you want to add additional nodes.

This already makes it nicer to work with docker and ROS writing python code, but we still have to restart the docker container manually, it would be nice if we could watch the file system for changes and restart our ros-node automatically, so lets do that next!

# Part 3: (optional) automatically reloading using watchdog
For this we use the watchdog-package to check if the node changed and automatically restart it.

In this part we extend on the previous tutorial and make use the functionality of docker volumes.

Because the docker container has its own folder structure and we now overwrite and add single files you might notice that it gets hard to keep track of this structure in your head. Remember that you can attach a bash to the container and check with "ls" to look at the file system inside the container.

TODO: Try for yourself
1. change into the `part3`-folder and run `docker compose build`. We need to install the `python3-watchdog` dependency. As its good practice to install files that do not change easily like your base dependencies first we put it in the beginning of the Dockerfile-publishing, besides that the Dockerfile looks the same as in the last part.
1. run `docker compose up`. Take a look at the first lines of the log you should see a print like this telling you that an observer has been started: `[publishing-1] [INFO] [1748688244.627064228] [publishing]: Watchdog starting observer.`
1. change the `publishing/node.py` file inside the part3-folder. This will restart the node.
1. In a new terminal run `docker compose run --rm publishing bash` to start a bash in a new container instance. This allows you to inspect files inside the container

Take a look at the docker-compose.yml file. There you'll find that the dev_watch.py-file overwrites the main.py we copied during the build of our image.

Also always make sure that the file names are correct, if you mistype the file name on your host system docker will create a folder for it and try to mount that folder into the docker image.

In the next part we will take a look at devContainer so we can run commands directly from inside the VS Code terminal inside the docker container to execute ros2 commands quickly and increase our productivity even more.

# Part 4: Basic ROS with docker in a devContainer
For development purposes we have a third docker image where we put both packages in one image for the dev-container.

# Part 5: Camera stream with RVIZ and C++
TODO: rviz and ros2 bag
TODO: recompile?

# Part 6: Include a webserver to control a robot
TODO: include https://github.com/brean/svelte-robot-control

# Part 7: Deploy to a robot
On your robot you want to know exactly what is running. Its often the case that, after testing and deploying to your machine you make changes specifically for your robot on it. These changes often do not find their way back into documentation or your code base.

TODO: 5.1 build package, push to robot (with and without publishing to dockerhub)

It is good practice to build your code in a docker image and only deploy that to the robot. If you deploy to a mobile ARM-based chip you want to cross-compile for that system.

TODO: 5.2 cross-compile for ARM64



