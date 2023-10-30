# Autonomous Bicycle - Python Enviorment

# Base image of container
FROM python:3.10.6-slim

# Set working directory of container
WORKDIR /usr/app/

# Add other modules to working directroy
COPY . .

# TODO: understand why we need this
ENV PYTHONPATH "${PYTHONPATH}:/usr/app/src"

# Install flit allow flit to install packages as root
RUN pip install flit
ENV FLIT_ROOT_INSTALL=1

# Install pip dependencies
COPY requirements.txt .
RUN pip install -r requirements.txt

# Install packages
WORKDIR /usr/app/src/qlearning
RUN flit install --symlink
WORKDIR /usr/app/src/state_pred
RUN flit install --symlink

# Reset working directory of container
WORKDIR /usr/app/

# GUI backend for python
RUN apt-get update --fix-missing  # Not sure why we need --fix-missing now
RUN apt-get install -y tk         # Required by Matplotlib for GUI

# Run on container start (not when using docker-compose run, however)
CMD [ "python", "src/main.py" ]
