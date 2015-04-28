export EMC_ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export EMC_DIR=$EMC_ENV_DIR/..

# --------------------------------------------------------------------------------

export EMC_SYSTEM_DIR=$EMC_DIR/system
export EMC_ROS_DISTRO=indigo

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
        git -C $EMC_ENV_DIR pull
    fi

    # Run the installer / updater
    $EMC_ENV_DIR/setup/emc-update-impl.bash

    # Source the updated environment
    source $EMC_ENV_DIR/setup.bash
}