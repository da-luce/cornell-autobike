# Setup

## Installing Docker

### MacOS

1. If not already installed, install [Homebrew](https://brew.sh/), a package
   manager for MacOS and Linux
2. Install Docker: `brew install --cask docker`
3. Open the newly installed _Docker Desktop_ application
   1. You should be able to find it in the application folder
   2. Otherwise, use Spotlight search (<kbd>⌘</kbd> + <kbd>space</kbd>) and
      search for "Docker"
4. Select "Use recommended settings"
5. Skip remaining steps if possible

### Linux

1. See [Docker install documentation](https://docs.docker.com/engine/install/)
   (untested)

### Windows

1. See [Docker install documentation](https://docs.docker.com/engine/install/)
   (untested)

## Running Docker

1. Set your working directory to the top level of the Q-learning repository
2. To run a command, use `docker-compose run nav --rm [COMMANDS]`
   1. This command will create an image and container (if necessary) and execute
      the given commands
   2. The `--rm` option deletes the container that was created to run the
      commands on exit
   3. Example: `docker-compose run nav --rm python main.py` will run main.py 🙂

## Tips

1. Create an alias in your `.bashrc` to avoid getting carpal tunnel

   ```bash
   alias nav='docker-compose run nav --rm'
   ```

2. The working directory of the navigation image (defined in
   `Dockerfile-navigation`) is set to `/usr/app`, which is bound as a volume to
   the entire Q-learning repository stored locally on the host (this way code
   changes do not require new image build, see `docker-compose.yml`)

   If you are working primarily in a specific module, it can be helpful to set
   your working directory to something else, e.g.

   ```text
   docker-compose run --rm --workdir /usr/app/src/state_pred/ nav python sim.py
   ```
