# Part 8: Using docker for microros development (FreeRTOS on Raspberry Pi Pico)
If you want to develop your own realtime microros-firmware the VSCode devcontainer could also be helpful, take a look at the [MicroROS Firmware for Huginn](https://github.com/brean/microros_firmware_huginn).
The devContainer in that project installs all required dependencies for microcontroller development without the need to install it locally, paths for the required SDKs are set so VS-Code can show you helpful error messages and code-autocompletion.

It clones the [MicroROS Raspberrypi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git) and overwrites its CMakeLists.txt in the Dockerfile. Without devContainer you would fork the `micro_ros_raspberrypi_pico_sdk`-repository and change the `CMakeLists.txt` and/or `pico_micro_ros_example.c`.

You just change the code and build it inside the devcontainer and then copy the `.uf2`-file to your pico. Take a look at the `compose.yml` file to see how volumes are used to support the deveopment.

Note that MicroROS only provides a limited number of services (normally just one in the generic configuration!)