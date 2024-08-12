# Autonomous Bicycle - Python Enviorment

# Base image
FROM python:3.10.6-slim

# Update package list and fix missing dependencies
RUN apt-get update --fix-missing

# Set working directory of container
WORKDIR /usr/app/
ENV PYTHONPATH="/usr/app/src:${PYTHONPATH}"
COPY . .

# OSMnx dependencies
RUN apt-get install -y \
    gdal-bin=3.2.2+dfsg-2+deb11u2 \
    libgdal-dev=3.2.2+dfsg-2+deb11u2 \
    g++=4:10.2.1-1

USER root

# Install Python dependencies defined in pyproject.toml
RUN pip install .

# GUI backend for python (required by Matplotlib)
RUN apt-get install -y tk=8.6.11+1

# Clean up cached package lists to reduce image size
RUN rm -rf /var/lib/apt/lists/*

USER 1001

# Run on container start (not when using docker-compose run, however)
CMD ["/bin/bash"]
