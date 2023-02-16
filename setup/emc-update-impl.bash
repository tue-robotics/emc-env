#! /bin/bash

err_report()
{
   exit_code=$? 
   echo -e "error(${exit_code}) on line $(caller)" >&2
   trap - ERR
   exit ${exit_code}
}

trap err_report ERR

# --------------------------------------------------------------------------------

function _make_sure_installed
{
    local pkgs_to_install
    local dpkg_query
    # shellcheck disable=SC2016
    dpkg_query=$(dpkg-query -W -f '${package} ${status}\n' 2>/dev/null)
    # shellcheck disable=SC2048
    for pkg in $*
    do
        # Check if pkg is not already installed dpkg -S does not cover previously removed packages
        # Based on https://stackoverflow.com/questions/1298066
        if ! grep -q "^$pkg install ok installed" <<< "$dpkg_query"
        then
            pkgs_to_install="$pkgs_to_install $pkg"
        fi
    done

    if [ -n "$pkgs_to_install" ]
    then
        echo "Going to install: $pkgs_to_install"

        # Wait for apt-lock first (https://askubuntu.com/a/375031)
        i=0
        tput sc
        while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1
        do
            case $((i % 4)) in
                0 ) j="-" ;;
                1 ) j="\\" ;;
                2 ) j="|" ;;
                3 ) j="/" ;;
            esac
            tput rc
            echo -en "\r[$j] Waiting for other software managers to finish..."
            sleep 0.5
            ((i=i+1))
        done

        # shellcheck disable=SC2086
        sudo apt-get install -y -q $pkgs_to_install
    fi
}

# --------------------------------------------------------------------------------

function _git_clone_or_update
{
    local repo_url=$1
    local dest=$2

    if [ ! -d "$dest" ]
    then
        git clone "$repo_url" "$dest"
    else
        git -C "$dest" pull
    fi
}

# --------------------------------------------------------------------------------

# Install ROS
if [ ! -d /opt/ros/"$EMC_ROS_DISTRO" ]
then
    if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]
    then
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

        curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

        sudo apt-get update
    fi

    # Install basic ROS packages. All other packages will be installed using tue-rosdep
    if [ "$EMC_ROS_DISTRO" != kinetic ] && [ "$EMC_ROS_DISTRO" != melodic ]
    then
        pv=3
    fi
    sudo apt-get install -y ros-"$EMC_ROS_DISTRO"-ros-base cmake python${pv}-catkin-pkg python${pv}-empy python${pv}-nose python${pv}-rosdep python${pv}-setuptools libgtest-dev build-essential

    if [ ! -d /etc/ros/rosdep ]
    then
        sudo rosdep init || true # make sure it always succeeds, even if rosdep init was already called
    fi

    rosdep update
fi

# Source ROS
# shellcheck disable=SC1090
source /opt/ros/"$EMC_ROS_DISTRO"/setup.bash

_make_sure_installed g++ git subversion

# Install EMC framework

# 1) Set-up Catkin workspace
if [ ! -d "$EMC_SYSTEM_DIR"/src ]
then
    mkdir -p "$EMC_SYSTEM_DIR"/src
fi

# 2) Download packages
_git_clone_or_update https://github.com/tue-robotics/emc_system "$EMC_SYSTEM_DIR"/src/emc_system
_git_clone_or_update https://github.com/tue-robotics/catkin_lint_cmake "$EMC_SYSTEM_DIR"/src/catkin_lint_cmake
_git_clone_or_update https://github.com/tue-robotics/emc_simulator "$EMC_SYSTEM_DIR"/src/emc_simulator
_git_clone_or_update https://github.com/tue-robotics/geolib2 "$EMC_SYSTEM_DIR"/src/geolib2
_git_clone_or_update https://github.com/tue-robotics/code_profiler "$EMC_SYSTEM_DIR"/src/code_profiler
_git_clone_or_update https://github.com/husarion/rosbot_description.git "$EMC_SYSTEM_DIR"/src/rosbot_description

if [ "$ROBOT_REAL" != true ]
then
    # Simbot specific packages
    _git_clone_or_update https://github.com/husarion/rosbot_description.git "$EMC_SYSTEM_DIR"/src/rosbot_description
else
    # Robot specific packages
    _git_clone_or_update https://github.com/tue-robotics/mrc_hero_bringup "$EMC_SYSTEM_DIR"/src/mrc_hero_bringup
fi

# 3) Install dependencies
_make_sure_installed python3-catkin-tools libassimp-dev ros-"${EMC_ROS_DISTRO}"-cv-bridge ros-"${EMC_ROS_DISTRO}"-image-geometry ros-"${EMC_ROS_DISTRO}"-map-server ros-"${EMC_ROS_DISTRO}"-message-generation ros-"${EMC_ROS_DISTRO}"-message-runtime ros-"${EMC_ROS_DISTRO}"-nav-msgs ros-"${EMC_ROS_DISTRO}"-robot-state-publisher ros-"${EMC_ROS_DISTRO}"-roscpp ros-"${EMC_ROS_DISTRO}"-rviz ros-"${EMC_ROS_DISTRO}"-shape-msgs ros-"${EMC_ROS_DISTRO}"-tf2 ros-"${EMC_ROS_DISTRO}"-tf ros-"${EMC_ROS_DISTRO}"-xacro

# 4) Compile
if [[ -n "$CI" ]]
then
	# suppress status bar in CI
	catkin build --workspace "$EMC_SYSTEM_DIR" -DCATKIN_ENABLE_TESTING=OFF --no-status
else
	catkin build --workspace "$EMC_SYSTEM_DIR" -DCATKIN_ENABLE_TESTING=OFF
fi

# 5) Install the libraries
sudo cp "$EMC_SYSTEM_DIR"/devel/lib/libemc_system.so /usr/lib/libemc-framework.so
sudo cp "$EMC_SYSTEM_DIR"/src/emc_system/include/emc /usr/include/ -r

# 6) Install gtest for localization assignments
_make_sure_installed libgtest-dev

trap - ERR
