# Autonomous Bicycle - Python Enviorment

# Base image (Humble is the LTS version for Ubuntu 22.04)
FROM ros:humble

# Update package list and fix missing dependencies
RUN apt-get update --fix-missing
RUN apt update --fix-missing

# Set working directory of container
WORKDIR /usr/app/
ENV PYTHONPATH="/usr/app/src:${PYTHONPATH}"
COPY . .

# OSMnx dependencies
RUN apt-get install -y \
    gdal-bin \
    libgdal-dev \
    g++

USER root

# Install Python dependencies defined in pyproject.toml
RUN apt-get install -y python3-pip
RUN pip install .

# GUI backend for python (required by Matplotlib)
RUN apt-get install -y tk

# ROS visualization
# RUN apt install -y \
#    ros-humble-rviz2 \
#    ros-humble-gazebo-ros

# RUN apt-get update && apt-get install -y \
#     mesa-utils \
#     libgl1-mesa-glx \
#     libgl1-mesa-dri

RUN bash -c "source /opt/ros/humble/setup.bash && apt-get install -y ros-humble-rviz2"


# Automatically source the ROS 2 environment for every shell session
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Clean up cached package lists to reduce image size
RUN rm -rf /var/lib/apt/lists/*


USER 1001

# Run on container start (not when using docker-compose run, however)
CMD ["/bin/bash"]
