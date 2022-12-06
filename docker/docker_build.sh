#!/bin/bash
#set -e
#
#PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"
#DOCKER_ROOT=${PROJECT_ROOT}/"/docker"
#echo "Project Root: $PROJECT_ROOT"
#
#echo "Building Docker image..."
#docker build --tag rosbarebones ${DOCKER_ROOT}

docker build --tag rosbarebones -f melodic/Dockerfile ..