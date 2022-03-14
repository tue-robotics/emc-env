export EMC_ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export EMC_DIR=$EMC_ENV_DIR/..

# --------------------------------------------------------------------------------

export EMC_SYSTEM_DIR=$EMC_DIR/system

if [ -z "$EMC_ROS_DISTRO" ]
then
    source /etc/lsb-release

    if [ "$DISTRIB_ID" != "Ubuntu" ]
    then
        echo "[emc-env] Unsupported OS $DISTRIB_ID. Use Ubuntu."
        exit 1
    fi

    # Set ROS version
    case $DISTRIB_RELEASE in
        "16.04")
            EMC_ROS_DISTRO=kinetic
            echo "[emc-env] Detected ubuntu 16.04, using ROS Kinetic"
            ;;
        "18.04")
            EMC_ROS_DISTRO=melodic
            echo "[emc-env] Detected ubuntu 18.04, using ROS Melodic"
            ;;
        "20.04")
            EMC_ROS_DISTRO=noetic
            echo "[emc-env] Detected ubuntu 20.04, using ROS Noetic"
            ;;
        *)
            echo "[emc-env] Ubuntu $DISTRIB_RELEASE is unsupported. Use either 16.04, 18.04 or 20.04"
            exit 1
            ;;
    esac
fi

export EMC_ROS_DISTRO

if [ -f $EMC_SYSTEM_DIR/devel/setup.bash ]
then
    source $EMC_SYSTEM_DIR/devel/setup.bash
fi

# --------------------------------------------------------------------------------

function emc-update
{
    if ! dpkg -s git &> /dev/null
    then
        echo "Going to install git"
        sudo apt-get install git
    fi

    # Update the installer / updater if not in CI
    if [[ -z "$CI" ]]
    then
        if [[ ! -d $EMC_ENV_DIR ]]
        then
            git clone https://github.com/tue-robotics/emc-env $EMC_ENV_DIR
        else
            git -C $EMC_ENV_DIR pull
        fi
    fi

    # Run the installer / updater
    $EMC_ENV_DIR/setup/emc-update-impl.bash

    # Source the updated environment
    source $EMC_ENV_DIR/setup.bash
}

# --------------------------------------------------------------------------------

alias hero-teleop='rosrun emc_simulator teleop.py taco'

alias mrc-update=emc-update

if [ "$ROBOT_REAL" = true ] ;
 then
  alias hero-start='roslaunch mrc_hero_bringup start.launch --screen'
else
  alias sshhero='ssh -A mrc@192.168.44.51'
  alias hero-core='export ROS_MASTER_URI=http://192.168.44.51:11311'
  alias mrc-sim='rosrun emc_simulator pico_simulator'
  alias mrc-viz='rosrun emc_system emc_viz'
fi
