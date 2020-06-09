#!/bin/bash
# Builds the firmware with the configuration specified by
# environment variables named CONFIG_...
# If DEPLOY is set, the deliverables are copied to Firmware/deploy/*
# with the suffix $DEPLOY
set -euo pipefail

THIS_DIR="$(dirname "$0")"
cd "$THIS_DIR"

# Treat warnings as errors
export CONFIG_STRICT=true

# Write all environment variables that start with "CONFIG_" to tup.config
rm -rdf build
mkdir -p build
env | grep ^CONFIG > tup.config
tup init
tup generate ./tup_build.sh
bash -xe ./tup_build.sh

# Deploy
if ! [ -z ${DEPLOY+x} ]; then
    mkdir -p deploy
    cp build/ODriveFirmware.elf deploy/ODriveFirmware_"$DEPLOY".elf
    cp build/ODriveFirmware.hex deploy/ODriveFirmware_"$DEPLOY".hex
fi
