# Autonomous Bicycle - Python Enviorment

# Base image of container
FROM python:3.10.6-slim
FROM gboeing/osmnx:latest

# Set working directory of container
WORKDIR /usr/app/
COPY . .
ENV PYTHONPATH="/usr/app/src:${PYTHONPATH}"

# Install pip dependencies defined in pyproject.toml
USER root
RUN pip install --upgrade pip setuptools wheel
RUN pip install .

# Install packages
# RUN flit install --symlink

# GUI backend for python
RUN apt-get update --fix-missing  # Not sure why we need --fix-missing now
RUN apt-get install -y tk         # Required by Matplotlib for GUI

USER 1001

# Run on container start (not when using docker-compose run, however)
CMD ["/bin/bash"]
