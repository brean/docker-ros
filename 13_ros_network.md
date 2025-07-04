# Part 13: ROS 2 and network
The biggest change from ROS 1 to ROS 2 is that instead of a basic Server-Client architecture ROS 2 uses a peer-to-peer communication model facilitated by [DDS (Data Distribution Service)](https://design.ros2.org/articles/ros_on_dds.html). Combined with the virtual network provided by Docker, the configuration of your network manager, the real network devices of your machine and the configuration of services in the network itself (like DHCP) this becomes a very complex setup quickly that can be hard to configure and debug.

TODO: Flow-Chart for issues

TODO: Docker network devices on linux and their issues (network manager that might interfer)

TODO: When to use network-mode: host

TODO: special IP addresses in the Docker network (The special 172.17.0.1 ip) (*Note on this magic IP: Some open WiFis use the same IP range, Deutsche Bahn is using the 172.X.X.X-network for their "WIFIonICE" Network for example, so if you don't get WiFi on the train configure your docker network to use another IP address pool like described [here (in German)](https://forum.ubuntuusers.de/topic/probleme-mit-dem-wifionice/#post-8964926).*)

TODO: Connecting MicroROS-Agent to ROS network (also link to this in chapter about microros)

TODO: Debugging issues with networking (own document?)

TODO: Discuss the issue if ROS 2 topic can be seen but no data

TODO: How and when does the ROS 2 middleware decide when to use memory and when does it 

TODO: the Future use zenoh everywhere!
