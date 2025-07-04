# Part 3: More on volumes (use case of automatically reloading using watchdog)
For automatically reloading file we use the Python 3 watchdog-package to check if the node changed and automatically restart it.

Because the docker container has its own folder structure and we now overwrite and add single files you might notice that it gets hard to keep track of this structure in your head. Remember that you can attach a bash to the container and check with "ls" to look at the file system inside the container.

**Try for yourself**
1. change into the `part3`-folder and run `docker compose build`. We need to install the `python3-watchdog` dependency so we can not use the one from part2 directly. As its good practice to install files that do not change easily like your base dependencies first we put it in the beginning of the Dockerfile-publishing, besides that the new Dockerfile-publishing looks the same as the one from the last part.
1. run `docker compose up` in the `part3`-folder. Take a look at the first lines of the log you should see a print like this telling you that an observer has been started: `[publishing-1] [INFO] [1748688244.627064228] [publishing]: Watchdog starting observer.`
1. change the `publishing/node.py` file inside the part3-folder. This will automatically restart the node. Because we watch the whole publishing-package you can change any file to restart, if you want to use it in your own projects you'll probably want to extend the logic in the `CodeChangeHandler::on_modified`-function to only reload when specific files are changed (e.g. only python files in the `publishing` and `launch`-subfolders).
1. In a new terminal run `docker compose run --rm publishing bash` to start a bash in a new container instance. This allows you to inspect files inside the container, use `ls` and `cd` to take a look around the `/ws/`-folder, which is your whole workspace. Because we did a `--symlink-install` the files from the `build`-folder link to your local `/ws/src/publishing/`-files where we have overwritten with our local volume mounts.

```bash
root@8e117851d90b:/ws/# ls -al /ws/build/publishing/launch/publishing.launch.py
lrwxrwxrwx 1 root root 42 May 31 10:20 /ws/build/publishing/launch/publishing.launch.py -> /ws/src/publishing/launch/publishing.launch.py
```
5. Press <kbd>Ctrl</kbd>+<kbd>c</kbd> to exit the docker container.
1. As always run `docker compose down` for cleanup.

Note that each container runs as its own instance, so the container running bash you just started and stopped is independent from your other subscription and publishing container and their file systems. They all share a few files from the volumes but all other files only exist while the container is running! So if you change any other file, add new files or delete files that are not linked to your file system via volume and you exit the container your changes are gone. If you start a new container with either `docker compose run`, `docker run` or `docker compose up` this new container gets created from the image as it was created from the Dockerfile, it has no memory of the container that came before.

When you look at the `compose.yml`-file, you'll see that we overwrite the launch file with a local file of the same name, the only change is that this launch file automatically restarts the container, we also overwrite main.py with our `watchdog_dev.py`-file and finally we add a new node.py-file that includes a copy of our old subscriber-node.

So we can use this to link any file or folder of the host system to any file or folder inside the docker container. Always make sure that the file names are correct, if you mistype the file name in your compose.yml on your host system for example, docker will create a folder for it and try to mount that folder into the docker image.

In the next part we will take a look at Dev Container so we can run commands directly from inside the VS Code terminal inside the docker container to execute ros2 commands quickly and increase our productivity even more.
