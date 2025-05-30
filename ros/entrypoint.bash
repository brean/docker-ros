#!/bin/bash

source  /ws/install/setup.bash
if [ $# -eq 0 ]; then
  echo "No command provided to entrypoint, starting /bin/bash to keep container alive."
  exec /bin/bash # Or exec sleep infinity
else
  # Execute the command passed to this entrypoint script
  exec "$@"
fi