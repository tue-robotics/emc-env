export EMC_ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export EMC_DIR=$EMC_ENV_DIR/..

# --------------------------------------------------------------------------------

export EMC_SYSTEM_DIR=$EMC_DIR/system

# Default ROS distro is indigo
[ -n "$EMC_ROS_DISTRO" ] || export EMC_ROS_DISTRO=indigo

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

    # Update the installer / updater
    if [ ! -d $EMC_ENV_DIR ]
    then
        git clone https://github.com/tue-robotics/emc-env $EMC_ENV_DIR
    else
        git --git-dir=$EMC_ENV_DIR/.git --work-tree=$EMC_ENV_DIR  pull
    fi

    # Run the installer / updater
    $EMC_ENV_DIR/setup/emc-update-impl.bash

    # Source the updated environment
    source $EMC_ENV_DIR/setup.bash
}

# --------------------------------------------------------------------------------

alias emc-sim='rosrun emc_simulator pico_simulator'
alias pico-teleop='rosrun emc_simulator teleop.py'
alias emc-viz='rosrun emc_system emc_viz'

alias pico-core='export ROS_MASTER_URI=http://192.168.2.81:11311'
alias taco-core='export ROS_MASTER_URI=http://192.168.2.82:11311' 
alias sshpico='ssh emc@192.168.2.81'
alias sshtaco='ssh emc@192.168.2.82'
