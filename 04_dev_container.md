# Part 4: Basic ROS with docker in a Dev Container
To have the full [VS Code](https://code.visualstudio.com/docs/devcontainers/create-dev-container) integration to use development tools like a local python debugger we have a third docker image where we put both packages in one image for the dev-container.

There is already a Dev Container setup for this repository, you find it in form of the file `.devcontainer/devcontainer.json`. This is the minimalistic setup that just points to the compose.yml file in the root folder of the repository to use the pubsubdev-server defined in the file.
For development we also install some ms-python extensions for VS Code so we can use debugpy to set a breakpoint.

The docker compose file itself includes volumes to the `publishing` and `subscribing` folders which both get installed inside the docker container. Because we want to have a bit more control we do not use the automatic reloading feature from part 3 so we have to run the ros2 commands to start subscribing and publishing manually.

Note that we also overwrite the command to run `sleep` infinitely. This is needed as it keeps the container alive so VS Code can attach to it. Attaching to a container is different from running a new container, we only use the container to synchronize files, it does not spawn a new container.

**Try for yourself**
1. Klick on the blue "><"-Icon in the bottom-left of the VS Code window.
1. Select "Reopen in DevContainer"
1. Open the file `src/publishing/publishing/main.py` and click on the play-button in the top-right. This starts the publisher in your terminal. Just let it running. Note that you started the node as simple ros-application without any ros setup.
1. Open the file `src/subscribing/subscribing/main.py` and click left of line 19 to create a breakpoint when the listener-callback gets called.
1. Instead of pressing the play button press the small arrow next to it and select `Python Debugger: Debug python file`. Again this runs the main-script as python file not thorugh ros but because we are inside the dev container we are using the python environment of our ros system so rclpy can be found and both scripts should be able to talk to each other so you should get the thrown into the debugger immediately when a message gets received:

![](docs/images/breakpoint_in_dev_container.png)

6. To see that debugging works you can check the value of `msg.data` by hovering over it with your mouse.
1. You can now stop the debugger and exit the container again and switch back to the main repository inside your file system again by clicking on "File" > "Open Recent" > and click on the repository folder without "[Dev Container]" in the name (it should be the top entry in the list).

Instead of running the `src/publishing/publishing/main.py` file from visual studio you could have also used the terminal:
1. Go back inside the container (press on the blue "><"-Icon in the bottom-left of the VS Code window) and select "Reopen in Container" or select "File" > "Open Recent" and click on the folder with "[Dev Container]" in the name.
1. Right-click in the file list and click on "Open in Integrated Terminal"
1. In the terminal you can now run ROS-commands inside the attached ros container, so you can simply run `ros2 launch publishing publishing.launch.py` or `ros2 run publishing publishing` to start the publisher.

Because it is a symlink-install you can quickly change the main.py file before running the ros2-command and the changes will be applied directly.

This is the minimum basic setup for a dev-container, we will at another, still simple but more realistic setup in the next part.

For more details read about the [VS Code Python debugging documentation](https://code.visualstudio.com/docs/python/debugging).
