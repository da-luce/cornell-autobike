# Setup

## Installing Docker

### MacOS

1. If not already installed, install [Homebrew](https://brew.sh/), a package
   manager for MacOS and Linux

2. Install Docker: `brew install --cask docker`

3. Open the newly installed _Docker Desktop_ application (should be located in
   application folder, otherwise use spotlight search)

4. Choose "Use recommended settings" and enter your password

5. Proceed to follow the installation process

See [Docker install documentation](https://docs.docker.com/engine/install/) for
more information.

## Running Docker

1. Ensure the working directory is the top level of the Q-learning repository

2. To build the image, run `docker build -t nav .`

3. A new image called "nav" should now be listed under Images in Docker Desktop

4. To run the image, use `docker run nav`
