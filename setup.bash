export EMC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source $EMC_DIR/setup/emc-update.bash

if [ -f $EMC_SYSTEM_DIR/devel/setup.bash ]
then
    source $EMC_SYSTEM_DIR/devel/setup.bash
fi
