# autodocking_jazzy_tb4

Repository for Autodocking with tb4 in Jazzy Docker Container

# Part 1: How to clone the repo and build the image using the files (Dockerfile and others), and then running it as a container

# Cloning Instructions:

To clone from Github:

```bash

git clone https://github.com/Naveensani/autodocking_jazzy_tb4.git

```
# Building Instructions:

To build the Image (Can take a while) (Use Image name of your preference):

```bash

cd autodocking_jazzy_tb4/

docker build -t naveensani22/ros2_jazzy_turtlebot:1.1.1 .

```
naveensani22/ros2_jazzy_turtlebot:1.1.1 name for the image is helpful if you are planning to upload this image that is build into Dockerhub. Here naveensani22 is my Dockerhub username. Any name can be used for this image that is about to be build.
And the image that is build can be seen using:

```bash
docker images
```

# Running Instructions:

First we have to give permission to Docker for display:

```bash
 xhost +local:docker
```
To Run this image as a container (Please use the name of the container that you want after --name and use the name of image that you named at the end. And give all the devices,display and permissions as required):

```bash

docker run -it \
  --name turtlebot_autodocking_container \
  --memory="8g" \
  --memory-swap="8g" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:1.1.1

```

After this command, you should see the terminal of the container that is running now. And to exit the container use:

```bash
exit
```

And to later use this same container, use (use the name of the container that you named):

```bash
docker start -ai turtlebot_autodocking_container
```

************

# Part 2: Doing docker pull for images and running it as a container

```bash
docker pull naveensani22/ros2_jazzy_turtlebot:1.1.1
```
After this, the image that is pulled can be seen using:

```bash
docker images
```
# Running Instructions:

To Run this image as a container (Please use the name of the container that you want after --name and use the name of image pulled. And give all the devices,display and permissions as required):

```bash

docker run -it \
  --name turtlebot_autodocking_container_1 \
  --memory="8g" \
  --memory-swap="8g" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  naveensani22/ros2_jazzy_turtlebot:1.1.1

```

After this command, you should see the terminal of the container that is running now. And to exit the container use:

```bash
exit
```

And to later use this same container, use (use the name of the container that you named):

```bash
docker start -ai turtlebot_autodocking_container_1
```

************



# Documentation of how to do all of things, that we figured out while trying to do things (Refer Part 4 to get recommended method)

# Part 3: The Whole path I took (Refer the dockerfile and the folder to see the commands I missed and the scripts respectively)


```bash
cd ~/turtlebot4_ws/src/
cd tb4_apriltag_bringup_cpp

mkdir -p launch worlds config

cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp/worlds

touch depot_with_apriltag.sdf

```
```bash
cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp/config

touch apriltag_params.yaml

```
```bash
cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp/src

touch detected_dock_pose_publisher.cpp

```
```bash
cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp

# Edit CMakeLists.txt

```
```bash
cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp

# Edit package.xml

```
```bash
cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp/launch

touch tb4_simulation_with_apriltag.launch.py


```
```bash
cd ~/turtlebot4_ws

colcon build --packages-select tb4_apriltag_bringup_cpp

source install/setup.bash


ros2 launch tb4_apriltag_bringup_cpp tb4_simulation_with_apriltag.launch.py

```

For docking server:-

```bash
sudo apt update 
sudo apt install ros-jazzy-opennav-docking

cd ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp/config

# Copy the default nav2 params
cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml nav2_params_with_docking.yaml

# Now edit it to add docking configuration
nano nav2_params_with_docking.yaml

nano ~/turtlebot4_ws/src/tb4_apriltag_bringup_cpp/launch/tb4_simulation_with_apriltag.launch.py
```

# Part 4: Notes on the Commands when launching autodocking
```bash
cd ~/turtlebot4_ws
colcon build --packages-select tb4_apriltag_bringup_cpp
source install/setup.bash

# Launch (this will now use our params with docking config)
ros2 launch tb4_apriltag_bringup_cpp tb4_simulation_with_apriltag.launch.py

```
```bash
In Another Terminal (To see the FPV from the camera): 

ros2 run image_view image_view --ros-args -r image:=rgbd_camera/image_rect
```
```bash
In Another Terminal (To dock the robot):
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{use_dock_id: true, dock_id: 'home_dock', dock_type: 'simple_charging_dock1', max_staging_time: 1000.0, navigate_to_staging_pose: true}"
```
```bash
In Another Terminal (To undock the robot):

ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{dock_type: 'simple_charging_dock1', max_undocking_time: 13.0}"

```
************
