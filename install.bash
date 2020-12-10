export EMC_DIR=~/.emc
export EMC_ENV_DIR=$EMC_DIR/env

if ! dpkg -s git &> /dev/null
then
    echo "Going to install git"
    sudo apt-get install git
fi

# Install / update the installer / updater
if [[ ! -d $EMC_ENV_DIR ]] && [[ -z "$CI" ]]
then
    git clone https://github.com/tue-robotics/emc-env $EMC_ENV_DIR
elif [[ -n "$CI" ]]
then
    mkdir -p $EMC_DIR
    cp -r . $EMC_ENV_DIR
else
    git -C $EMC_ENV_DIR pull
fi

# Source the updated environment
source $EMC_ENV_DIR/setup.bash

# Add sourcing the environment to ~/.bashrc (if not already there)
! grep 'Source the EMC environment' ~/.bashrc -q && echo -e "\n# Source the EMC environment\nsource $EMC_ENV_DIR/setup.bash" >> ~/.bashrc

# Run the installer / updater
emc-update
