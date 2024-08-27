# ROS setup

# Environment variables should be set via the docker file

# Environment variables for the core ROS2 libraries and tools
source /opt/ros/${ROS_DISTRO}/setup.bash

# Aliases

# Common tools
alias lint='(cd $WORKDIR && pylint --rcfile=pyproject.toml src/)'
alias format='(cd $WORKDIR && black --config pyproject.toml .)'
alias format_check='(cd $WORKDIR && black --check --config pyproject.toml .)'
alias type_check='(cd $WORKDIR && mypy --exclude=src/unrosified --config-file=pyproject.toml src/)'
# Don't need pytest, as it grabs correct information running anywhere in the project

# Colcon
# Careful with subshells!
alias build='(cd $WORKDIR && colcon build) && source $WORKDIR/install/setup.bash'
alias coltest='(cd $WORKDIR && colcon test --event-handlers console_direct+)'
alias clean='(cd $WORKDIR && rm -rf build/ install/ log/)'

# Other
alias python=python3
alias ros_test='(cd $WORKDIR && ros2 run demo_nodes_cpp talker)'
