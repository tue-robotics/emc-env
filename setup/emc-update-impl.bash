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

        if [[ ! -f /tmp/emc_apt_get_updated ]]
        then
            sudo apt-get update -qq
            touch /tmp/emc_apt_get_updated
        fi

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
if [[ ! -d /opt/ros/"${EMC_ROS_DISTRO}" ]]
then
    if [[ -f /etc/apt/sources.list.d/ros.list ]]
    then
        sudo rm -f /etc/apt/sources.list.d/ros.list*
    fi

    source /etc/os-release  # Get the UBUNTU_CODENAME

   # Check whether universe is enabled
   # The following regex checks for a 'deb' line in /etc/apt/sources.list that matches the current Ubuntu codename
   # and includes the 'universe' component. It handles possible variations in the line format.
   if ! grep -h ^deb /etc/apt/sources.list 2>/dev/null | grep -P "${UBUNTU_CODENAME}[a-z\-]* (?:[a-z ]*(?:[a-z]+(?: [a-z]+)*)) universe" -q
   then
       sudo add-apt-repository universe
       rm -f /tmp/emc_apt_get_updated
   fi

    CURL_ARGS=("-H" "Accept: application/vnd.github+json")
    if [[ -n ${GITHUB_TOKEN} ]]
    then
        CURL_ARGS+=("-H" "Authorization: Bearer ${GITHUB_TOKEN}")
    fi
    newest_version=$(curl "${CURL_ARGS[@]}" -sL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -m1 '"tag_name":' | sed -E 's/.*"tag_name": *"([^"]+)".*/\1/')
    
    [[ ${newest_version} != "null" ]] || { echo "Failed to retrieve latest ros-apt-source version" >&2; exit 1; }

    ros_apt_source_pkg_name="ros-apt-source"
    curl -fL -o /tmp/${ros_apt_source_pkg_name}.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${newest_version}/${ros_apt_source_pkg_name}_${newest_version}.${UBUNTU_CODENAME}_all.deb"
    # Verify the downloaded .deb file is a valid Debian package before installing
    if ! dpkg-deb --info /tmp/${ros_apt_source_pkg_name}.deb > /dev/null 2>&1
    then
        echo "Downloaded package is not a valid .deb file. Aborting." >&2
        exit 1
    fi
    sudo dpkg -i /tmp/${ros_apt_source_pkg_name}.deb
    rm -f /tmp/emc_apt_get_updated

    # Install basic ROS packages. All other packages will be installed using tue-rosdep
    _make_sure_installed ros-"${EMC_ROS_DISTRO}"-ros-base cmake python3-catkin-pkg python3-empy python3-nose python3-rosdep python3-setuptools libgtest-dev build-essential

    if [[ ! -d /etc/ros/rosdep ]]
    then
        sudo rosdep init || true # make sure it always succeeds, even if rosdep init was already called
    fi

    rosdep update
fi

# Source ROS
# shellcheck disable=SC1090
source /opt/ros/"$EMC_ROS_DISTRO"/setup.bash

_make_sure_installed g++ git

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
fi

# 3) Install dependencies
_make_sure_installed python3-catkin-tools libassimp-dev ros-"${EMC_ROS_DISTRO}"-cv-bridge ros-"${EMC_ROS_DISTRO}"-image-geometry ros-"${EMC_ROS_DISTRO}"-map-server ros-"${EMC_ROS_DISTRO}"-message-generation ros-"${EMC_ROS_DISTRO}"-message-runtime ros-"${EMC_ROS_DISTRO}"-nav-msgs ros-"${EMC_ROS_DISTRO}"-robot-state-publisher ros-"${EMC_ROS_DISTRO}"-joint-state-publisher ros-"${EMC_ROS_DISTRO}"-roscpp ros-"${EMC_ROS_DISTRO}"-rviz ros-"${EMC_ROS_DISTRO}"-shape-msgs ros-"${EMC_ROS_DISTRO}"-tf2 ros-"${EMC_ROS_DISTRO}"-tf ros-"${EMC_ROS_DISTRO}"-xacro

# 4) Compile
if [[ -n "$CI" ]]
then
    # suppress status bar in CI
    catkin build --workspace "${EMC_SYSTEM_DIR}" -DCATKIN_ENABLE_TESTING=OFF --no-status
else
    catkin build --workspace "${EMC_SYSTEM_DIR}" -DCATKIN_ENABLE_TESTING=OFF
fi

# 5) Install the libraries
sudo cp "$EMC_SYSTEM_DIR"/devel/lib/libemc_system.so /usr/lib/libemc-framework.so
sudo cp "$EMC_SYSTEM_DIR"/src/emc_system/include/emc /usr/include/ -r

trap - ERR
