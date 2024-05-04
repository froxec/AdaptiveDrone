#!/bin/bash

# Source the setup.bash script
source /workspace/install/setup.bash

# Start a bash shell
exec /bin/bash

# and add this at the end
exec "$@"
