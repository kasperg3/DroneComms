# Use an official ROS 2 base image
FROM ros:humble

# Install any dependencies if necessary
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools

# Copy the requirements file

# Install the Python dependencies
RUN pip3 install --no-cache-dir pyserial 

# COPY ./communication/requirements.txt /tmp/
#RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Copy your ROS 2 package to the container
COPY ./communication /ros2_ws/src/communication
# Copy the communication_interface package to the container
COPY ./common_interface /ros2_ws/src/common_interface

# Set the workspace
WORKDIR /ros2_ws/src
# Build the ROS 2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select common_interface communication"

# Set the default command to run your node
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/src/install/setup.bash && ros2 run communication driver --ros-args --params-file /config/config.yaml"]