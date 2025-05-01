# Build-time arguments
ARG ROS_DISTRO=humble

# Base image (Humble is the LTS version for Ubuntu 22.04)
# IMPORTANT: pin to a specific hash!
FROM --platform=linux/amd64 ros:${ROS_DISTRO}

# Set working directory
ENV WORKDIR /usr/local/autobike
WORKDIR $WORKDIR

# Update package list and fix missing dependencies
RUN apt-get update --fix-missing

# Basic tools
RUN apt-get install -y vim

# Install pip (updating is important!)
RUN apt-get install -y python3-pip && pip3 install --upgrade pip

# Setup ROSboard
RUN apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp && \
    pip3 install tornado==6.4.1 simplejpeg==1.7.4 && \
    git clone https://github.com/dheera/rosboard.git /usr/local/rosboard

# OSMnx dependencies
RUN apt-get install -y \
    gdal-bin=3.4.1+dfsg-1build4 \
    libgdal-dev=3.4.1+dfsg-1build4 \
    g++=4:11.2.0-1ubuntu1

# Install xeyes to test NoVNC
RUN apt-get update && apt-get install -y x11-apps

# Install rviz2
RUN apt install -y \
    ros-humble-rviz2 \
    libogre-1.12-dev

# Webots
RUN apt install -y ros-humble-webots-ros2
# install 2025a version manually
RUN apt-get update && apt-get install -y wget && \
    wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb && \
    apt install -y ./webots_2025a_amd64.deb && \
    rm ./webots_2025a_amd64.deb

# pointcloud_to_grid package
RUN apt-get update
RUN apt install -y ros-humble-pcl-ros

# GUI backend for python (required by Matplotlib)
RUN apt-get install -y python3.10-tk

# Copy Python project files and install dependencies
ENV PYTHONPATH="$WORKDIR:${PYTHONPATH}"
COPY pyproject.toml .
# added another flag to avoid transforms3d from crashing
RUN pip install --ignore-installed transforms3d==0.4.2 .

# Run the container as a non-root user for better security
ARG USERNAME=bichael
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME:autobike" | chpasswd && \
    adduser $USERNAME sudo

ENV HOME=/home/$USERNAME
RUN chown -R $USERNAME:$USERNAME $HOME

# Add shell env to .bashrc (overwritten by volume)
COPY shell_env.sh $HOME/shell_env.sh
RUN echo "source \$HOME/shell_env.sh" >> $HOME/.bashrc

# Clean up cached package lists to reduce image size
RUN rm -rf /var/lib/apt/lists/*

# Copy the rest of the application files
COPY . .

USER $USERNAME

# Run on container start (not when using docker-compose run, however)
CMD ["/bin/bash"]
