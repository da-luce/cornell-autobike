# set base image (host OS)
FROM python:3.10.6-slim

COPY requirements.txt .
RUN pip install -r requirements.txt

# Copy all the files from the current directory to the Docker image
COPY . .

# Run on container start
CMD [ "python", "./main.py" ]
