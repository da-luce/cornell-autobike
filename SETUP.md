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

### Linux

See [Docker install documentation](https://docs.docker.com/engine/install/)

## Running Docker

1. Ensure the working directory is the top level of the Q-learning repository

2. To run a one off command, use `docker-compose run nav --rm [COMMANDS]`
   1. This command runs the `nav` service defined in `docker-compose.yml`, which
      is an image built from `Dockerfile-navigation` with the specified pip
      packages
   2. The `--rm` option deletes the container that was created to run the
      commands on exit
   3. Example: `docker-compose run nav --rm python main.py` will run main.py :)
   4. Create an alias in your `.bashrc` to avoid getting carpal tunnel:

   ```bash
   alias nav='docker-compose run nav --rm'
   ```
