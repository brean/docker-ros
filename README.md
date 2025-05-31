# Dev Container and ROS 2
This repository should teach you how to work with docker and ros in a modern development setup using Visual Studio Code and dev container. It also provides a source of example files and links to examples and projects that you can use to build more complex robot systems.

This tutorial is directed towards ROS 2 (mostly python) developers who want to accelerate their development process, ROS 2 basics are assumed.

## Installation
You need
1. [VS Code](https://code.visualstudio.com/) (If you want to use [Codium](https://vscodium.com/) you can use DevPod, however it requires some more steps to setup and is a bit harder to debug).
1. docker ([docker-ce](https://docs.docker.com/engine/install/) recommended)
   1. don't forget to do the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) (creating the `docker`-group and add yourself)
1. (optional) [docker-nvidia-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) if you have an nvidia-gpu and don't want software rendering for 3D tools like [rviz](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html).
1. Beginner to intermediate ROS 2 and python knowledge, some docker basics. The [Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) on the ROS 2 website are a good start be aware of a steep learning curve if you are new to ROS!

# Part 1: Docker and ROS (without devcontainer)
Even if you want to use devContainer it is best practice to have a setup that also runs without it so people can run your container even if something does not work with their setup or if they want to use another IDE/Editor and not VS Code. I prefere `docker compose` to configure my docker container so I don't have to run long `docker`-commands.

First lets take a look at a simple ROS 2 setup with two custom ros packages in the `ros`-folder, it contains one package that publishes and a second package that has a subscriber to these messages. Its taken from the [tutorial for writing a basic publisher and subscriber in python.](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) So it should be easy to understand however both single subscriber and publisher scripts are accompanied by a some files to create individual packages and launch files for both as you might find in a more complex ROS 2 setup.

The application for this could be one ROS-package that you install on your PC while the other is running on your robot (often times accompanied by a third package that contains shared ROS-messages on both but as we are only sending basic strings we use the `std_msgs`-package that comes with our ROS 2 installation).
We will look at more realistic use cases in the later parts.

Because they are separated services they both can be seen as individual machines separated from your computer (your host machine) with their own file systems and networking. Docker compose automatically connects both to the same virtual network so we don't need to configure that.

Open a terminal, cd into the `part1`-folder of this repository and type `docker compose up publishing subscribing`. It will build and run the 2 docker images and start 2 docker container, one for publishing and one for subscribing, one outputs to the local docker network and the other is receiving these messages.
The source for both packages can be found in the `ros/`-folder

When you press CTRL+C both container should exit, if you can't wait for them to stop, press CTRL+C again or close the terminal window.

Afterwards you need to run `docker compose down` in the `part1`-folder to make sure both docker container and the network are removed.

This is nice as you now have a ROS 2 setup system-independent 
All files are build in your container, however you need to rebuild the docker container with `docker compose build` every time you change any file in your ros-packages and the files from the docker image differ from your local files. 

**Try for yourself**: 
1. Build the container using `docker compose build` in the `part1`-folder
1. Change code in the `ros`-folder, for example change line 18 in the file `ros/publishing/publishing/main.py` from `msg.data = 'Hello World: %d' % self.i` to `msg.data = 'Sending: %d' % self.i`
1. restart the docker container without building by just running `docker compose up` in the `part1`-folder. You will still see the `Hello World`-messages.
1. re-run the `docker compose build`-command and then restart with `docker compose up`, now you should see the updated data getting published.
1. Run `docker compose down` inside the `part1`-folder for cleanup.

# Part 2: Docker Volumes
Using docker volumes you can increase your development speed: you don't have to re-run the `docker compose build` command all the time when you change code. Note that this only helps you for interpreted languages like python, for C/C++ applications it makes more sense to rerun the docker compose build-process manually (as the code has to compile anyway) and use a compile cache.

**Try for yourself**:
1. change into the `part2`-folder and run `docker compose build`
1. run `docker compose up` and keep it running
1. change the code, for example instead of "I heard: " you could print out something like "I received: " (in `ros/subscribing/subscribing/main.py`)
1. save the file. You'll notice that the output of your docker compose command still continues with "I heared:"
1. stop both docker container with CTRL+C and directly recreate it with `docker compose up`.
1. Now you see the "I received:" - without the need of rebuilding the docker container!
1. Run `docker compose down` inside the `part2`-folder for cleanup.

Note that we build the packages with `colcon` as part of the `docker compose build`-process in the Dockerfile. This means that only the code at the time you build the image gets copied into your docker image. We overwrite the code later with the volumes when we start the container. This works because we build the package as `symlink-install`, so the files that are copied into the install-folder of your workspace (which can be found at `/ws/install` in your image) link to the files of your `/ws/subscribing` or `/ws/publishing` folders. If you remove the volume your old code will be used again, as it is stored in the image. This also means that when you have bigger changes, change your structure or add new packages you need to rebuild the image.

So volumes help with smaller changes while developing your code but you still need to rebuild the image when dependencies change or when you want to add additional nodes. You don't have to overwrite the full folder, you can also overwrite individual files as we see in the next part.

Using volumes already makes it nicer to work with docker and ROS writing python code, but we still have to restart the docker container manually, it would be nice if we could watch the file system for changes and restart our ros-node automatically, so lets do that next!

# Part 3: More on volumes (use case of automatically reloading using watchdog)
For automatically reloading file we use the Python 3 watchdog-package to check if the node changed and automatically restart it.

Because the docker container has its own folder structure and we now overwrite and add single files you might notice that it gets hard to keep track of this structure in your head. Remember that you can attach a bash to the container and check with "ls" to look at the file system inside the container.

**Try for yourself**
1. change into the `part3`-folder and run `docker compose build`. We need to install the `python3-watchdog` dependency so we can not use the one from part2 directly. As its good practice to install files that do not change easily like your base dependencies first we put it in the beginning of the Dockerfile-publishing, besides that the new Dockerfile-publishing looks the same as the one from the last part.
1. run `docker compose up` in the `part3`-folder. Take a look at the first lines of the log you should see a print like this telling you that an observer has been started: `[publishing-1] [INFO] [1748688244.627064228] [publishing]: Watchdog starting observer.`
1. change the `publishing/node.py` file inside the part3-folder. This will automatically restart the node. Because we watch the whole publishing-package you can change any file to restart, if you want to use it in your own projects you'll probably want to extend the logic in the `CodeChangeHandler::on_modified`-function to only reload when specific files are changed (e.g. only python files in the `publishing` and `launch`-subfolders).
1. In a new terminal run `docker compose run --rm publishing bash` to start a bash in a new container instance. This allows you to inspect files inside the container, use `ls` and `cd` to take a look around the `/ws`-folder, which is your whole workspace. Because we did a `--symlink-install` the files from the `build`-folder link to your local `/ws/publishing/`-files where we have overwritten with our local volume mounts.

```bash
root@8e117851d90b:/ws# ls -al /ws/build/publishing/launch/publishing.launch.py
lrwxrwxrwx 1 root root 42 May 31 10:20 /ws/build/publishing/launch/publishing.launch.py -> /ws/publishing/launch/publishing.launch.py
```
5. Press CTRL+C to exit the docker container.
1. As always run `docker compose down` for cleanup.

Note that each container runs as its own instance, so the container running bash you just started and stopped is independent from your other subscription and publishing container and their file systems. They all share a few files from the volumes but all other files only exist while the container is running! So if you change any other file, add new files or delete files that are not linked to your file system via volume and you exit the container your changes are gone. If you start a new container with either `docker compose run`, `docker run` or `docker compose up` this new container gets created from the image as it was created from the Dockerfile, it has no memory of the container that came before.

When you look at the `docker-compose.yml`-file, you'll see that we overwrite the launch file with a local file of the same name, the only change is that this launch file automatically restarts the container, we also overwrite main.py with our `watchdog_dev.py`-file and finally we add a new node.py-file that includes a copy of our old subscriber-node.

So we can use this to link any file or folder of the host system to any file or folder inside the docker container. Always make sure that the file names are correct, if you mistype the file name in your docker-compose.yml on your host system for example, docker will create a folder for it and try to mount that folder into the docker image.

In the next part we will take a look at devContainer so we can run commands directly from inside the VS Code terminal inside the docker container to execute ros2 commands quickly and increase our productivity even more.

# Part 4: Basic ROS with docker in a devContainer
To have the full VS Code integration to use development tools like a local debugger we have a third docker image where we put both packages in one image for the dev-container.

TODO: Setup and start devcontainer and debug something

TODO: extend devContainer with some basic ros2 package, python-package and other dependencies

# Part 5: Camera stream with RVIZ and C++
TODO: rviz and ros2 bag

TODO: recompile C++-code?

# Part 6: Include a webserver to control a robot
TODO: include https://github.com/brean/svelte-robot-control and gazebo

# Part 7: Dependency Management
TODO: discuss where to install what dependencies and show options like autoproj, vcs, ...

# Part 8: Deploy to a robot
On your robot you want to know exactly what is running. Its often the case that, after testing and deploying to your machine you make changes specifically for your robot on it. These changes often do not find their way back into documentation or your code base.

TODO: 5.1 build package, push to robot (with and without publishing to dockerhub)

# Part 9: Cross-compile for ARM
It is good practice to build your code in a docker image and only deploy that to the robot. If you deploy to a mobile ARM-based chip you want to cross-compile for that system.

TODO: 5.2 cross-compile for ARM64

# Part 10: The DFKI Docker Image Development
TODO: Link to basics on https://github.com/dfki-ric/docker_image_development

TODO: create repo using docker_image_development and integrate it in devContainer.
