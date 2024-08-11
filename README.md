# Autobike Software

[![codecov](https://codecov.io/gh/da-luce/Q-Learning/graph/badge.svg?token=DG2EJ0SJPB)](https://codecov.io/gh/da-luce/Q-Learning)
![Build Status](https://github.com/da-luce/Q-Learning/actions/workflows/build.yml/badge.svg)

- [Autobike Software](#autobike-software)
  - [Git Basics](#git-basics)
    - [Git Resources](#git-resources)
  - [Running Code with Docker](#running-code-with-docker)
    - [Installation](#installation)
      - [MacOS](#macos)
      - [Linux](#linux)
      - [Windows](#windows)
    - [Building an Image](#building-an-image)
    - [Running in a Container](#running-in-a-container)
    - [X Window Forwarding](#x-window-forwarding)
      - [MacOS](#macos-1)
      - [Linux](#linux-1)
      - [Windows](#windows-1)
    - [Testing](#testing)
  - [Best Practices](#best-practices)
    - [Formatting and Linting](#formatting-and-linting)
    - [Directory Structure and Testing](#directory-structure-and-testing)
  - [Architecture](#architecture)

## Git Basics

- **Clone the Repository:**
  - To start working on the project, clone the repository to your local machine:

    ```bash
    git clone git@github.com:da-luce/Q-Learning.git
    ```

- **Navigate to the Repository:**
  - Change your current directory to the project directory:

    ```bash
    cd Q-Learning
    ```

- **Create a New Branch:**
  - Before making changes, create a new branch off of `main`:

    ```bash
    git checkout -b <feature-name> main
    ```

    - Replace `<feature-name>` with a descriptive name for your feature (without the `<>`).
    - Example: `git checkout -b add-user-auth main`

- **Make Changes and Commit:**
  - Work on your feature or changes. When ready, stage and commit your changes:

    ```bash
    git add .
    git commit -m "Descriptive message about what you changed"
    ```

- **Push Your Branch to GitHub:**
  - After committing your changes, push your branch to GitHub:

    ```bash
    git push origin <feature-name>
    ```

- **Create a Pull Request:**
  - Go to the repository on GitHub and navigate to the [Pull Requests page](https://github.com/da-luce/Q-Learning/pulls).
  - Create a pull request to merge your branch into the `dev` branch.

### Git Resources

- [Git Basics Documentation](https://git-scm.com/doc)
- [Pro Git Book](https://git-scm.com/book/en/v2)
- [GitHub Guides](https://guides.github.com/)

## Running Code with Docker

### Installation

#### MacOS

1. If not already installed, install [Homebrew](https://brew.sh/), a package
   manager for MacOS and Linux
2. Install Docker

   ```text
   brew install --cask docker
   ```

3. Open the newly installed _Docker Desktop_ application
   1. You should be able to find it in the application folder
   2. Otherwise, use Spotlight search (<kbd>⌘</kbd> + <kbd>space</kbd>) and
      search for "Docker"
4. Select "Use recommended settings"
5. Skip remaining steps if possible

#### Linux

1. See [Docker install documentation](https://docs.docker.com/engine/install/)
   (ask Ari)

#### Windows

1. See [Docker install documentation](https://docs.docker.com/engine/install/)
   (untested)

### Building an Image

If it is your first time running the code with Docker or there have been changes made to the Dockerfile, you need to build the Docker image:

```bash
docker build -t autobike:latest .
```

Running `docker image ls` should now show this image, alongside any others you have built:

```text
REPOSITORY       TAG       IMAGE ID       CREATED         SIZE
autobike         latest    121bfa1d2778   2 minutes ago   3.67GB
```

> [!NOTE]
> Images can be deleted with `docker image rm`

### Running in a Container

To create a container and run code, you can use one of the following techniques:

- Interactive shell: `docker run -it --rm -v "$(pwd):/usr/app" --user root autobike`
- One off command: `docker run -it --rm -v "$(pwd):/usr/app" --user root autobike <your command>`

> [!NOTE]
> The volume is mounted such that changes to your local code are immediately reflected in the container.**

Example of a one off command:

```bash
docker run -it --rm -v "$(pwd):/usr/app" --user root autobike python src/qlearning/main.py
```

> [!NOTE]
> Spinning up a new container isn't free--it can take a few moments. As such, an interactive shell is often the preferred method of development.

> [!TIP]
> Create an alias in your `.bashrc` (or whatever shell you are using) to avoid getting carpal tunnel
>
> ```bash
> alias autobike="docker run -it --rm -v '$(pwd):/usr/app' --user root autobike"
> ```
>
> A quick way to do this if you are using Bash:
>
> ```bash
> echo <above_command> >> ~/.bashrc
> ```
>
> Or Zsh (default on MacOS):
>
> ```bash
> echo <above_command> >> ~/.zshrc
> ```

### X Window Forwarding

#### MacOS

1. Install [XQuartz](https://www.xquartz.org/) via Homebrew

    ```text
    brew install --cask xquartz
    ```

2. Logout and login of your Mac to activate XQuartz as default X11 server
3. Open XQuartz

    ```text
    open -a XQuartz
    ```

4. Go to Security Settings (Menu Bar -> XQuartz -> Settings -> Security Settings)
   and ensure that "Allow connections from network clients" is on
5. Restart your laptop (FIXME: this step may not be necessary?)
6. Allow X11 forwarding from localhost via xhost

    ```text
    xhost + localhost
    ```
    
> [!IMPORTANT]
> This must be run in the xterm window opened by default by XQuartz _every time_ XQuartz is started

7. Add the following option whenever you run `docker run`:

    ```text
    --env DISPLAY=host.docker.internal:0
    ```

    Example:

    ```text
    docker run -it --rm -v "$(pwd):/usr/app" --user root --env DISPLAY=host.docker.internal:0 autobike python src/state_pred/bike_sim.py
    ```

#### Linux

In _theory_ this should be easier as most distros already use Xwindows,
but I know for a fact that the steps differ compared to Mac.

May have to bind this volume:

```text
--volume="$HOME/.Xauthority:/root/.Xauthority:rw"
```

May have to set

```text
--net=host
```

or

```text
--add-host=host.docker.internal:host-gateway
```

May have to bind x11 socket:

```text
--volume /tmp/.X11-unix:/tmp/.X11-unix
```

May have to set DISPLAY to

`--env DISPLAY=unix$DISPLAY` or `--env DISPLAY=$DISPLAY`

I'm not sure. Stack Overflow has a sea of possibilities.

May the odds be ever in your favor...

#### Windows

No clue.

### Testing

1. Follow the above guide for running code in Docker containers
2. Run `pytest` within a container in the top level dir of the repo

> [!NOTE]
> This `/usr/app/` given how our volume is mounted

## Best Practices

### Formatting and Linting

We use [black](https://github.com/psf/black) for formatting along with [flake8](https://flake8.pycqa.org/en/latest/) and [pylint](https://pypi.org/project/pylint/) for linting. If you are using [VS Code](https://code.visualstudio.com/), [vscode settings](.vscode/settings.json) should automatically setup everything you need. Just make sure you have the [recommended extensions](./.vscode/extensions.json) installed.

For other IDEs, there may be extensions provided for these tools, or you could just use the CLI equivalents. Make sure to pass the `pyproject.toml` file as an arg (e.g. `--rcfile=pyproject.toml` or `--config=pyproject.toml`) to use the same formatting and linting settings as the rest of the project. Examples (run from the top level of the repo):

- **Linting:** `pylint --rcfile=pyproject.toml src/`
- **Test:** `pytest`
- **Formatting:** `black --config pyproject.toml .`
- **Type checking:** `mypy --config-file=pyproject.toml src/`

### Directory Structure and Testing

```text
.
├── src/
│   ├── moduleA
│   ├── moduleB/
│   │   ├── __init__.py
│   │   └── README.md
│   └── main.py
└── test/
    ├── test_moduleA.py
    └── test_moduleB.py
```

- Each module should contain an `__init__.py` file along with a `README.md` which
  contains basic documentation on how one should use the file:
  - Exported constants or functions
  - Implementation documentation as seen fit
- Each module should have a corresponding `test_moduleX.py` file containing `pytest` tests
  - Perhaps, add a small [mermaid](https://mermaid.live) diagram

## Architecture

TODO: add overarching mermaid diagram
