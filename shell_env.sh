#!/bin/bash

# ROS setup

# Environment variables for the core ROS2 libraries and tools
source /opt/ros/${ROS_DISTRO}/setup.bash

# Environment of our specific workspace--locally built packages
source $WORKDIR/install/setup.bash

# Aliases

# Common tools
alias lint='(cd $WORKDIR && pylint --rcfile=pyproject.toml src/)'
alias format='(cd $WORKDIR && black --config pyproject.toml .)'
alias format_check='(cd $WORKDIR && black --check --config pyproject.toml .)'
alias type='(cd $WORKDIR && mypy --config-file=pyproject.toml src/)'
# Don't need pytest, as it grabs correct information running anywhere in the project

# Colcon
alias build='(cd $WORKDIR && colcon build)'
alias ctest='(cd $WORKDIR && colcon test --event-handlers console_direct+)'

# Other
alias python=python3
alias ros_test='(cd $WORKDIR && ros2 run demo_nodes_cpp talker)'
