# Build-time arguments
ARG ROS_DISTRO=humble
ARG ROSBOARD_PORT=8888

# Base image (Humble is the LTS version for Ubuntu 22.04)
FROM ros:${ROS_DISTRO}

WORKDIR /usr/app/

# Update package list and fix missing dependencies
RUN apt-get update --fix-missing

ENV PYTHONPATH="/usr/app/src:${PYTHONPATH}"
COPY pyproject.toml .

# OSMnx dependencies
RUN apt-get install -y \
    gdal-bin \
    libgdal-dev \
    g++

RUN sudo apt-get install -y python3-pip
RUN pip install --upgrade pip
RUN python3 -m pip install  .

COPY . .

USER root

# GUI backend for python (required by Matplotlib)
RUN apt-get install -y tk

# Setup ROSboard
RUN apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp && \
    pip3 install tornado simplejpeg && \
    git clone https://github.com/dheera/rosboard.git
EXPOSE ${ROSBOARD_PORT}

# Source the ROS 2 environment for every shell session
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && apt-get install -y ros-humble-rviz2"
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Clean up cached package lists to reduce image size
RUN rm -rf /var/lib/apt/lists/*

USER 1001

# Run on container start (not when using docker-compose run, however)
CMD ["/bin/bash"]
