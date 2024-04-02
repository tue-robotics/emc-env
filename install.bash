export EMC_DIR=~/.emc
export EMC_ENV_DIR=$EMC_DIR/env

if ! dpkg -s curl &> /dev/null
then
    echo "Going to install curl"
    sudo apt-get install -y -q curl
fi

if ! dpkg -s git &> /dev/null
then
    echo "Going to install git"
    sudo apt-get install -y -q git
fi

if ! dpkg -s python-is-python3 &> /dev/null
then
    echo "Going to install python-is-python3"
    sudo apt-get install -y -q python-is-python3
fi

# Install / update the installer / updater
if [[ ! -d $EMC_ENV_DIR ]] && [[ -z "$CI" ]]
then
    git clone git@github.com:/tue-robotics/emc-env $EMC_ENV_DIR
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
