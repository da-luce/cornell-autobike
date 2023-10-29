# Autonomous Bicycle - Python Enviorment

# Base image of container
FROM python:3.10.6-slim

# Set working directory of container
WORKDIR /usr/app/

# Add other modules to working directroy
COPY /src .

# Install flit
RUN pip install flit

# Allow flit to install packages as root
ENV FLIT_ROOT_INSTALL=1

# Install pip dependencies
COPY requirements.txt .
RUN pip install -r requirements.txt

# Go to the package directory and then install
WORKDIR /usr/app/qlearning
RUN flit install --symlink

WORKDIR /usr/app/state_pred
RUN flit install --symlink

# Set working directory of container
WORKDIR /usr/app/

# GUI backend for python
RUN apt-get update --fix-missing  # Not sure why we need --fix-missing now
RUN apt-get install -y tk         # Required by Matplotlib for GUI

# Run on container start (not when using docker-compose run, however)
CMD [ "python", "src/main.py" ]
