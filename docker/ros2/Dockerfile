FROM ros:humble

# Install required ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    ros-humble-web-video-server \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set DDS implementation
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Create workspace
RUN mkdir -p /ros2_ws/launch

WORKDIR /ros2_ws

# Source ROS2 on entry
ENTRYPOINT ["/ros_entrypoint.sh"]
