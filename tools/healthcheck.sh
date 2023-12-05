#!/bin/bash

HEALTHCHECK_FILE="/var/tmp/health_status.txt"


# Now check the health status
if [ -f "$HEALTHCHECK_FILE" ]; then
    status=$(cat "$HEALTHCHECK_FILE")
    if [ "$status" == "healthy" ]; then
        exit 0
    else
        exit 1
    fi
else
    echo "Healthcheck file still not found. There may be an issue with the node."
    exit 1
fi
