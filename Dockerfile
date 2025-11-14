# 1. Base image
FROM ros:jazzy

# 2. Install system dependencies and tools
RUN apt update && \
    apt upgrade -y


# 4. Create and initialize TurtleBot4 workspace
RUN mkdir -p /root/turtlebot4_ws/src
WORKDIR /root/turtlebot4_ws/src

# 5. Clone the TurtleBot4 simulator repo
RUN git clone https://github.com/turtlebot/turtlebot4_simulator.git -b jazzy

# 6. Install ROS 2 package dependencies
WORKDIR /root/turtlebot4_ws
RUN rosdep update && \
    rosdep install --from-path src -yi

# 7. Build the workspace
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/turtlebot4_ws/install/setup.bash" >> /root/.bashrc
    
    
RUN apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb* \
    ros-jazzy-nav2-route \
    ros-jazzy-image-proc \
    ros-jazzy-apriltag-ros \
    ros-jazzy-opennav-docking
    
    
WORKDIR /root
RUN git clone --branch harmonic https://github.com/rickarmstrong/gazebo_apriltag.git

# 5. Copy custom package
COPY tb4_apriltag_bringup_cpp /root/turtlebot4_ws/src/tb4_apriltag_bringup_cpp

RUN echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/root/gazebo_apriltag/" >> /root/.bashrc

WORKDIR /root/turtlebot4_ws
#RUN rosdep update && \
#    rosdep install --from-path src -yi
    
# 7. Build the workspace
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# 10. Set working directory and default entrypoint
WORKDIR /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch","tb4_apriltag_bringup_cpp","tb4_simulation_with_apriltag.launch.py"]
