# Build-time arguments
ARG ROS_DISTRO=humble

# Base image (Humble is the LTS version for Ubuntu 22.04)
FROM ros:${ROS_DISTRO}

# Set working directory
WORKDIR /usr/local/autobike

# Update package list and fix missing dependencies
RUN apt-get update --fix-missing

# Install pip (updating is important!)
RUN apt-get install -y python3-pip && pip3 install --upgrade pip

# Setup ROSboard
RUN apt-get install -y \
    ros-${ROS_DISTRO}-demo-nodes-cpp && \
    pip3 install tornado simplejpeg && \
    git clone https://github.com/dheera/rosboard.git /usr/local/rosboard

# OSMnx dependencies
RUN apt-get install -y \
    gdal-bin \
    libgdal-dev \
    g++

# GUI backend for python (required by Matplotlib)
RUN apt-get install -y python3.10-tk

# Copy Python project files and install dependencies
ENV PYTHONPATH="/usr/app/src:${PYTHONPATH}"
COPY pyproject.toml .
RUN pip3 install .

# Run the container as a non-root user for better security
ARG USERNAME=bichael
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

ENV HOME=/home/$USERNAME
ARG RC=$HOME/.bashrc

RUN chown -R $USERNAME:$USERNAME $HOME

# Source the ROS 2 environment for every shell session
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $RC

# Aliases
RUN echo "alias ros_test='ros2 run demo_nodes_cpp talker'" >> $RC
RUN echo "alias lint='pylint --rcfile=pyproject.toml src/'" >> $RC
RUN echo "alias pytest='pytest'" >> $RC
RUN echo "alias format='black --config pyproject.toml .'" >> $RC
RUN echo "alias type='mypy --config-file=pyproject.toml src/'" >> $RC
RUN echo "alias python=python3" >> $RC

# Clean up cached package lists to reduce image size
RUN rm -rf /var/lib/apt/lists/*

# Copy the rest of the application files
COPY . .

USER $USERNAME

# Run on container start (not when using docker-compose run, however)
CMD ["/bin/bash"]
