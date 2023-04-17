#!/usr/bin/bash
# build and run development docker image
set -eox pipefail

# get path to here
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# build and tag docker image
IMAGE=openvdb-voxl-aggregator:latest
docker build -t $IMAGE -f $SCRIPTPATH/Dockerfile .

# drop into a gpu-enabled container via rocker
rocker --nvidia --x11 --volume $SCRIPTPATH:/workspace -- $IMAGE byobu
