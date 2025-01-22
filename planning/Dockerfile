# Use an official ROS 2 base image
FROM ros:humble

# Install any dependencies if necessary
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools

# Copy the requirements file

# Install the Python dependencies
RUN pip3 install --no-cache-dir pyserial trajallocpy==0.0.13 shapely extremitypathfinder[numba] matplotlib numpy geojson

# Copy your ROS 2 package to the container
COPY ./planning /ros2_ws/src/planning
# Copy the communication_interface package to the container
COPY ./common_interface /ros2_ws/src/common_interface

WORKDIR /ros2_ws/src
# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Set the default command to run your node
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/src/install/setup.bash && ros2 run planning planner --ros-args --params-file /config/config.yaml"]