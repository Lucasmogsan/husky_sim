# Clearpath Husky Simulation with Realsense RGB-D Sensor

Remember to:
- [ ] Get submodules
- [ ] Read documentation
- [ ] Build and source the overlay if it doesn't work (should've already been done in the Dockerfile though)
- [ ] Have fun



# Husky
Links:
- [General Clearpath Documentation](https://www.clearpathrobotics.com/assets/guides/noetic/husky/index.html)
- [GitHub](https://github.com/husky/husky?tab=readme-ov-file)

## Husky real robot

Networking (in this case using a mobile hotspot with IP `172.20.10.1/24`) - inspiration from this [blogpost](https://chantrapornchai.medium.com/tips-in-setting-ros-networking-3d8ed0a2e43f):
1. Connect Husky to network and set the connection priority to the lowest (to make sure it connects to the one intended)
1. Check the Husky IP (e.g. `ifconfig`): In this example it has inet / IP address `172.20.10.2`.
1. Set up the ROS network configurations as below:
    On Robot PC:
    ```bash
    $ export ROS_IP=ip_of_this_machine
    $ export ROS_MASTER_URI=http://ip_of_this_machine:11311
    ```
    On remote PC:
    ```bash
    $ export ROS_IP=ip_of_this_machine
    $ export ROS_MASTER_URI=http://ip_of_master:11311
    ```
1. This is done by: Modifying `/etc/ros/setup.bash` on Husky: Add `export ROS_MASTER_URI=http://172.20.10.2:11311` and `export ROS_IP=172.20.10.2`
1. Connect remote PC (in this example with IP `172.20.10.3`) to same network and do:
    ```bash
    export ROS_MASTER_URI=http://172.20.10.2:11311
    export ROS_IP=172.20.10.3
    ```
1. Verify by running `rostopic list` on remote pc and `rostopic echo` some topic like `/husky_velocity_controller/cmd_vel`
1. if firewall is blocking the connection disable it `ufw disable`

In case the topics can be seen but not read (no output when using rostopic echo), the Husky ros nodes likely use a hostname for publishing.
1. See hostname used for Husky nodes:
    ```bash
    rosnode info <node-names>
    ```
1. Add the hostname to the remote PCs hostnames in `/etc/hosts`. In the husky example it should resolve the hostname `cpr-a200-0632` to its correct IP (`172.20.10.2`).
    ```bash
    172.20.10.2 cpr-a200-0632
    ```

### Husky w. ORB_SLAM

Setup on Husky (if not already done) - recommended to be done with screen, mouse, and keyboard:
1. Disable EKF for IMU and wheels (in `/etc/ros/melodic/ros.d/base.launch`)
1. Export URDF file with correct placement of realsense to `HUSKY_URDF_EXTRAS` variable (this is done by modifying `/etc/ros/setup.bash` which is being run by `/usr/sbin/ros-start` script)
1. If URDF is changed (or added) restart ROS by `sudo systemctl restart ros`
1. Clone `orb_slam3_ros`, `realsense_ros`, and `pointcloud_to_grid` repositories to the home directory
1. Pull the docker images from dockerhub (`lucasmogsan/orbslam3_ros` and `lucasmogsan/realsense_ros1`) 
1. Connect to a common network which can be used by remote PC as well.

Start Husky with ORB-SLAM:
1. Boot up the Husky (automatically starts ROS and the Husky drivers)
1. (ssh onto the Husky from remote PC - find the IP by getting on same wifi and use nmap e.g. `nmap -sn 172.20.10.0/24`)
1. (spin up container with ROS1 on remote pc and very same ROS_MASTER - Husky ROS_IP is set in `etc/ros/setup.bash`)
1. Spin up docker containers for `orb_slam3_ros` and `realsense_ros` by `docker compose up dev` from their respective directories.
1. Launch the realsense node `roslaunch realsense2_camera rs_camera.launch`
1. Start ORB-SLAM `roslaunch orb_slam3_ros rs_rgbd.launch`

Save ORB-SLAM trajectory:


### Husky w. realsense recording to bag

Setup on Husky (if not done already):
1. Clone `realsense_ros` repository to the home directory
1. Pull the docker image from dockerhub (`lucasmogsan/realsense_ros1`)

Record ros-bag with realsense camera:
1. Boot up Husky
1. Spin up docker container with ROS on remote PC
1. ssh onto Husky from remote PC
1. Spin up docker container `realsense_ros` from its directory
    ```bash
    cd realsense_ros1_docker/
    docker compose up dev
    ```
    In another terminal
    ```bash
    docker exec -it realsense_ros1_docker-dev-1 bash
    roslaunch realsense2_camera rs_camera.launch
    ```
1. Launch the realsense node `roslaunch realsense2_camera rs_camera.launch`
1. View from remote pc
    ```bash
    rqt
1. record rosbag from remote pc
    Specific topic (preferred)
    ```bash
    rosbag record <topic-names>
    rosbag record /camera/depth/image_rect_raw
    ```
    All topics (Note that newly published topics are discovered by periodically polling the master. rosbag record -a will likely miss initial messages published on any topic.)
    ```bash
    rosbag record -a
    ```


# Husky Simulator
[Husky Simulator Documentation](https://www.clearpathrobotics.com/assets/guides/noetic/husky/SimulatingHusky.html)


## Husky with ORB-SLAM3
1. Spin up both husky-sim and orb_slam3_ros containers
    ```bash
    cd <folders>
    docker compose up dev
    ```
1. launch the environment from husky_sim overlay_ws which also spawns the robot with the sensor (custom urdf).
    ```bash
    ./src/husky_custom/launch_office.sh
    ```
1. launch ORB-SLAM from orb_slam3_ros overlay_ws without any pre-defined map.
    ```bash
    roslaunch orb_slam3_ros rs_rgbd_sim.launch load_atlas:=false
    ```
1. launch keyboard teleoperation of husky from husky_sim overlay_ws
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

#### Generate and save ORB map and occupancy grid

Generate map using the `pointcloud_to_grid` package (in orb_slam3_ros overlay_ws) and use this for navigation using `ORB-SLAM` and `husky-navigation`:
1. launch pointcloud to grid from orb_slam3_ros overlay_ws.
    ```bash
    roslaunch pointcloud_to_grid create_occupancy_grid.launch
    ```
1. Save 3D ORB map `.osa` from ORB-SLAM (saves in the `ROS_HOME` folder (`~/.ros/` by default)).
    ```bash
    rosservice call /orb_slam3/save_map orbmap_construction_site
    ```
    1. Alternatively save by the following to save to the orb_slam3_ros folder which is mounted from your PC.
        ```bash
        rosservice call /orb_slam3/save_map ./../../../overlay_ws/src/orb_slam3_ros/maps/orbmap_construction_site
        ```
1. Save 2D occupancy grid from the dynamic `pointcloud_to_grid` reconfigure application.
    1. Specify path (from within the container, `/overlay_ws/src/orb_slam3_ros/maps/` is default), name (`orbmap_construction_site` is default) and click save in the reconfigure application.
    1. It saves a `.pgm` and associating `.yaml` file.

#### Use pre-mapped area for localization and navigation.

1. Make sure the occupancy `.pgm` and `.yaml` files are in `husky_navigation_custom/maps` and `amcl_husky.launch` includes correct path and file-name.
1. Make sure the ORB map is located in `orb_slam3_ros/maps` and `rs_rgbd_sim.launch` includes correct path and file-name.

1. launch ORB-SLAM from orb_slam3_ros overlay_ws with `load_atlas_from_file` pointing to the 3D ORB map `.osa`.
    ```bash
    roslaunch orb_slam3_ros rs_rgbd_sim.launch load_atlas:=true
    ```
    1.  ! Remember to specify the correct path to the map in the launch file.

1. launch navigation from husky_sim overlay_ws
    ```bash
    roslaunch husky_navigation_custom amcl_husky.launch
    ```


## Other simulation environments

Simulate Husky in an empty world. You can add new objects to this world using the Gazebo controls (Gazebo Tutorial - Building a World).
```bash
roslaunch husky_gazebo empty_world.launch
```

Simulate Husky in a Clearpath designed world. This is the base environment for the navigation tutorials. It will take some time to start, as the simulator will need to download resources from the Gazebo servers.
```bash
roslaunch husky_gazebo husky_playpen.launch
```


Pick your own world in which to simulate Husky (Using roslaunch with Gazebo).
```bash
roslaunch husky_gazebo husky_playpen.launch
```

Using RVIZ (requires not only `ros-${ROS_DISTRO}-husky-simulator` but also `ros-${ROS_DISTRO}-husky-desktop` to be installed)
```bash
roslaunch husky_viz view_robot.launch
```

## Husky Navigation examples
NB: `ros-${ROS_DISTRO}-husky-navigation` must be installed.


## Customize URDF Husky Robot and Gazebo Simulation

[GitHub](https://github.com/husky/husky_customization)

## Customized payloads

https://www.clearpathrobotics.com/assets/guides/noetic/husky/additional_sim_worlds.html#customizepayload


### Realsense RGB-D
Possible to simulate stereo depth camera using the Openni Kinect plugin.

Realsense `realsense2_camera` and `realsense2_description` is installed.

A customized URDF file is created for the sensor.

To add this to the Husky simulation, set the `HUSKY_URDF_EXTRAS` environment variable
```bash
export HUSKY_URDF_EXTRAS=/overlay_ws/src/husky_custom/thesis_custom/urdf/realsense.urdf.xacro
```

### LiDAR 2D scan

```bash
export HUSKY_LMS1XX_ENABLED=1
```



## Additional Worlds:
[Clearpath Documentation](https://www.clearpathrobotics.com/assets/guides/noetic/husky/additional_sim_worlds.html)


```bash
roslaunch cpr_office_gazebo office_world.launch platform:=husky
```

NB: The [cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo) package can be heavy so we've copied only the office world to the husky_custom repository.
To install all worlds add the [cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo) as submodule.





to use with noetic maybe see: https://gitlab.gbar.dtu.dk/s184915/34763-autonomous-marine-robotics/-/tree/main/ros_ws/src/bluerov2








# Other ROS related stuff

## Twist commands with keyboard

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## RViz
Open rviz with custom configuration file
```bash
rviz -d /overlay_ws/src/husky_custom/thesis_custom/rviz/simple_husky_rgbd.rviz
```



# General information

## Docker (How to run docker environment)

Install docker and docker compose

Docker:
https://docs.docker.com/engine/install/

Docker compose:
https://docs.docker.com/compose/install/

Build the image:
```bash
docker compose build dev
```

Run the container:
```bash
docker compose up dev
```

Connect to the container:
```bash
docker exec -it $NAME bash
```

Remove everything, including stopped containers and all unused images (not just dangling ones):
```bash
docker system prune -a
```

## Submodules
Clone the repo with submodules:
```bash
git clone --recursive git@github.com:Lucasmogsan/husky_sim.git
```

Alternatively clone the repo and then get the submodules afterwards:

```bash
git clone git@github.com:Lucasmogsan/husky_sim.git
```

```bash
git submodule update --init --recursive
```


The main repo has references to the submodules. If these submodules are modified, then the main repo may need to update these references in order to pull the latest data.
```bash
git submodule update --remote
```

This modifies the references in the main repo, and these changes needs to be committed and pushed.


Add:
```bash
cd packages
git submodule add git@github.com:Lucasmogsan/husky_custom.git
```
maybe with --force

Remove: Go to folder containing 
```bash
cd packages
git rm husky_custom/
```
maybe with -f