# Autonomous Bicycle - Python Enviorment

# Base image of container
FROM python:3.10.6-slim
FROM gboeing/osmnx:latest

# Set working directory of container
WORKDIR /usr/app/

# Add other modules to working directroy
COPY . .

# Install flit allow flit to install packages as root
RUN pip install flit
ENV FLIT_ROOT_INSTALL=1

# Install pip dependencies
COPY requirements.txt .
USER root
RUN pip install -r requirements.txt

# Install packages
RUN flit install --symlink

# GUI backend for python
RUN apt-get update --fix-missing  # Not sure why we need --fix-missing now
RUN apt-get install -y tk         # Required by Matplotlib for GUI
USER 1001

# Run on container start (not when using docker-compose run, however)
CMD [ "python", "src/main.py" ]
