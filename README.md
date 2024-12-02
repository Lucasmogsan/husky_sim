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

# Husky Simulator
[Husky Simulator Documentation](https://www.clearpathrobotics.com/assets/guides/noetic/husky/SimulatingHusky.html)

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
export HUSKY_URDF_EXTRAS=/overlay_ws/src/husky_custom/urdf/realsense.urdf.xacro
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






to use with noetic maybe see: https://gitlab.gbar.dtu.dk/s184915/34763-autonomous-marine-robotics/-/tree/main/ros_ws/src/bluerov2








# Other ROS related stuff

## Twist commands with keyboard

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## RViz
Open rviz with custom configuration file
```bash
rviz -d /overlay_ws/src/husky_custom/rviz/simple_husky_rgbd.rviz
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

This modifies the references in the main repo, and these changes needs to be comitted and pushed.