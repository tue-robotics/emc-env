#! /bin/bash

# --------------------------------------------------------------------------------

function _make_sure_installed
{
    local pkgs_to_install=
    for pkg in $@
    do
        if ! dpkg -s $pkg &> /dev/null
        then
            pkgs_to_install="$pkgs_to_install $pkg"
        fi
    done

    if [ -n "$pkgs_to_install" ]
    then
        echo "Going to install: $pkgs_to_install"
        sudo apt-get install -y $pkgs_to_install
    fi
}

# --------------------------------------------------------------------------------

function _git_clone_or_update
{
    local repo_url=$1
    local dest=$2

    if [ ! -d $2 ]
    then
        git clone $repo_url $dest
    else
        git -C $dest pull
    fi    
}

# --------------------------------------------------------------------------------

# Install ROS
if [ ! -d /opt/ros/$EMC_ROS_DISTRO ]
then
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

    sudo apt-get update

    # Install basic ROS packages. All other packages will be installed using tue-rosdep
    sudo apt-get install -y ros-$EMC_ROS_DISTRO-ros-base cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential

    sudo rosdep init || true # make sure it always succeeds, even if rosdep init was already called

    rosdep update
fi

# Source ROS
source /opt/ros/$EMC_ROS_DISTRO/setup.bash

_make_sure_installed g++ git subversion

# Install EMC framework

# 1) Set-up Catkin workspace
if [ ! -d $EMC_SYSTEM_DIR/src ]
then
    mkdir -p $EMC_SYSTEM_DIR/src
    catkin_make --directory $EMC_SYSTEM_DIR
    source $EMC_SYSTEM_DIR/devel/setup.bash
fi

# 2) Download packages
_git_clone_or_update https://github.com/tue-robotics/emc_system $EMC_SYSTEM_DIR/src/emc_system
_git_clone_or_update https://github.com/tue-robotics/emc_simulator $EMC_SYSTEM_DIR/src/emc_simulator
_git_clone_or_update https://github.com/tue-robotics/geolib2 $EMC_SYSTEM_DIR/src/geolib2
_git_clone_or_update https://github.com/tue-robotics/code_profiler $EMC_SYSTEM_DIR/src/code_profiler

# 3) Install dependencies
_make_sure_installed ros-$EMC_ROS_DISTRO-cv-bridge ros-$EMC_ROS_DISTRO-tf libassimp-dev ros-$EMC_ROS_DISTRO-message-runtime ros-$EMC_ROS_DISTRO-message-generation ros-$EMC_ROS_DISTRO-roscpp

# 4) Compile
catkin_make --directory $EMC_SYSTEM_DIR

# 5) Install the libraries
sudo cp $EMC_SYSTEM_DIR/devel/lib/libemc_system.so /usr/lib/libemc-framework.so
sudo cp $EMC_SYSTEM_DIR/src/emc_system/include/emc /usr/include/ -r
