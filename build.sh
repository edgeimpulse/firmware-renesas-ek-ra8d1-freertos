#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

TARGET=$1

if [ -z "$TARGET" ]; then
    TARGET="Debug"
fi

echo "Building"

if [ "$TARGET" == "SDRAM" ]; then
    /usr/bin/e2studio --launcher.suppressErrors -no-indexer -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /app/workspace -import ${SCRIPTPATH} -cleanBuild firmware-renesas-ek-ra8d1-freertos/SDRAM || true &
else
    TARGET="Debug"
    /usr/bin/e2studio --launcher.suppressErrors -no-indexer -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /app/workspace -import ${SCRIPTPATH} -cleanBuild firmware-renesas-ek-ra8d1-freertos/Debug || true &
fi

pid=$! # Process Id of the previous running command
while kill -0 $pid 2>/dev/null
do
    echo "Still building..."
    sleep 2
done

wait $pid
if [ -f /app/workspace/firmware-renesas-ek-ra8d1-freertos/${TARGET}/firmware-renesas-ek-ra8d1-freertos.hex ]; then
    echo "Building done"
    exit 0
else
    echo "Building failed"
    exit 1
fi
