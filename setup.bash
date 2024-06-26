export EMC_ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export EMC_DIR=$EMC_ENV_DIR/..
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
        "20.04")
            EMC_ROS_DISTRO=noetic
            echo "[emc-env] Detected ubuntu 20.04, using ROS Noetic"
            ;;
        "18.04")
            EMC_ROS_DISTRO=melodic
            echo "[emc-env] Detected ubuntu 18.04, using ROS Melodic"
            ;;
        *)
            echo "[emc-env] Ubuntu $DISTRIB_RELEASE is unsupported. Use 20.04, 18.04"
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
        sudo apt-get install -y -q  git
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
#export ROS_HOSTNAME=$HOSTNAME.local

alias mrc-teleop='rosrun emc_system teleop.py'

alias mrc-update=emc-update

export BOBO_IP='192.168.6.68'
export COCO_IP='192.168.6.186'

if [ "$ROBOT_REAL" == true ]
then
  alias hero-start='rosparam load $EMC_SYSTEM_DIR/src/emc_system/config/hero_mrc_config.yaml'
  alias rosbot-start='roslaunch rosbot_bringup start_emc.launch'
  alias bobo-start='roslaunch rosbot_bringup start_emc.launch name:=bobo'
  alias coco-start='roslaunch rosbot_bringup start_emc.launch name:=coco'

  alias define-map='rosrun map_server map_server'
else
  alias sshbobo='ssh -A -X husarion@$BOBO_IP'
  alias sshcoco='ssh -A -X husarion@$COCO_IP'
  alias sshhero='ssh -A -X mrc@192.168.44.51'
  alias bobo-core='export ROS_MASTER_URI=http://$BOBO_IP:11311'
  alias coco-core='export ROS_MASTER_URI=http://$COCO_IP:11311'
  alias hero-core='export ROS_MASTER_URI=http://192.168.44.51:11311'
  alias mrc-sim='rosrun emc_simulator simulator'
  alias sim-rviz='roslaunch emc_simulator viz.launch'
  alias mrc-open-door='rostopic pub --once /pyro/open_door std_msgs/Empty "{}"'
  alias mrc-speech='rosrun pico_talk speech_server.py'

  alias hero-rviz='roslaunch emc_system hero_rviz.launch'
  alias rosbot-rviz='roslaunch emc_system rosbot_rviz.launch'
  alias bobo-rviz='roslaunch emc_system rosbot_rviz.launch'
  alias coco-rviz='roslaunch emc_system rosbot_rviz.launch'
fi
