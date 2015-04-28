export EMC_DIR=~/.emc
export EMC_ENV_DIR=$EMC_DIR/env

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
source $EMC_DIR/env/setup.bash