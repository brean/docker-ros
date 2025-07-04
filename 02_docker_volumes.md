# Part 2: Docker Volumes
Using docker volumes you can increase your development speed: you don't have to re-run the `docker compose build` command all the time when you change code. Note that this only helps you for interpreted languages like python, for C/C++ applications it makes more sense to rerun the docker compose build-process manually (as the code has to compile anyway) and use a compile cache.

**Try for yourself**:
1. change into the `part2`-folder and run `docker compose build`
1. run `docker compose up` and keep it running
1. change the code, for example instead of "I heard: " you could print out something like "I received: " (in `ros/subscribing/subscribing/main.py`)
1. save the file. You'll notice that the output of your docker compose command still continues with "I heared:"
1. stop both docker container with <kbd>Ctrl</kbd>+<kbd>c</kbd> and directly recreate it with `docker compose up`.
1. Now you see the "I received:" - without the need of rebuilding the docker container!
1. Run `docker compose down` inside the `part2`-folder for cleanup.

Note that we build the packages with `colcon` as part of the `docker compose build`-process in the Dockerfile. This means that only the code at the time you build the image gets copied into your docker image. We overwrite the code later with the volumes when we start the container. This works because we build the package as `symlink-install`, so the files that are copied into the install-folder of your workspace (which can be found at `/ws/install` in your image) link to the files of your `/ws/src/subscribing` or `/ws/src/publishing` folders. If you remove the volume your old code will be used again, as it is stored in the image. This also means that when you have bigger changes, change your structure or add new packages you need to rebuild the image.

So volumes help with smaller changes while developing your code but you still need to rebuild the image when dependencies change or when you want to add additional nodes. You don't have to overwrite the full folder, you can also overwrite individual files as we see in the next part.

Using volumes already makes it nicer to work with docker and ROS writing python code, but we still have to restart the docker container manually, it would be nice if we could watch the file system for changes and restart our ros-node automatically, so lets do that next!
