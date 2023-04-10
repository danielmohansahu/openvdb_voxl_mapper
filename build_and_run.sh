#!/usr/bin/bash
# build and run development docker image
set -eo pipefail

# get path to here
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# build and tag docker image
IMAGE=openvdb-voxl-aggregator:latest
docker build -t $IMAGE -f $SCRIPTPATH/Dockerfile .

# drop into a gpu-enabled container via rocker
rocker run --nvidia --x11 -it --rm --volume $SCRIPTPATH:/workspace $IMAGE bash
