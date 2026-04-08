#!/bin/bash
set -eo pipefail
DEFAULT_HOST="$WO_HOST"
DEFAULT_DIR="$WO_DIR"

# Parse args for overrides
POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --hostname)
    export WO_HOST="$2"
    shift # past argument
    shift # past value
    ;;
        --dir)

    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    POSITIONAL+=("$1") # save it in an array for later
    shift # past argument
    ;;
esac
done
set -- "${POSITIONAL[@]}" # restore positional parameter

usage(){
  echo "Usage: $0 <command>"
  echo
  echo "This program helps to manage the setup/teardown of the docker containers for running ros2_robotica. We recommend that you read the full documentation of docker at https://docs.docker.com if you want to customize your setup."
  echo 
  echo "Command list:"
  echo "        start [options]         Start ros2_robotica"
  echo "        stop                    Stop ros2_robotica"
  echo "        down                    Stop and remove ros2_robotica's docker containers"
  echo "        rebuild                 Rebuild docker image and perform cleanups"
  echo "        open                    Open terminal to run scripts and whatever"
  echo ""
  #echo "Options:"
  #echo "       --hostname      <hostname>      Set the hostname that PointLearning will be accessible from (default: $DEFAULT_HOST)"
  #echo "       ---dir  <path>  Path where data will be persisted (default: $DEFAULT_DIR (docker named volume))"
  exit
}


run(){
  echo "$1"
  eval "$1"
}

start(){
  # 1. Allow X Server access (for X11)
  xhost +local:docker
  
  # 2. Basic X11/Qt Variables
  # 2. Basic X11/Qt Variables
  X_VARS="-e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
  
  # 3. Wayland Variables (Using the /run/user/$UID/ directory)
  WAYLAND_VARS="-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=/run/user/$(id -u) -v /run/user/$(id -u):/run/user/$(id -u)"

  # 4. Device and Networking Access
  NETWORK_DEVICE_ACCESS="--net=host --device=/dev/dri:/dev/dri"

  run "docker run --rm --name ros2_robotica --shm-size=256m \
    -v /home/sebastian/uba/Robotica-Movil-UBA/talleres:/root/ros2_ws/src/robotica \
    -v /home/sebastian/uba/Robotica-Movil-UBA/tp_final/modelo_omnidireccional:/root/ros2_ws/src/modelo_omnidireccional \
    -v /home/sebastian/uba/Robotica-Movil-UBA/tp_final/coppeliaSim:/root/coppeliaSim \
    ${NETWORK_DEVICE_ACCESS} \
    ${X_VARS} \
    ${WAYLAND_VARS} \
    -it -d ros2_robotica"
}

down(){
  run "docker stop ros2_robotica"
  run "docker rm ros2_robotica"
}

rebuild(){
  down
  run "docker build -t ros2_robotica . "
}

build(){
  run "docker build -t ros2_robotica ."
}

open(){
  run "docker exec -it ros2_robotica bash"
}


if [[ $1 = "start" ]]; then
        start
elif [[ $1 = "stop" ]]; then
        echo "Stopping ros2_robotica..."
        run "docker stop ros2_robotica"
elif [[ $1 = "down" ]]; then
        echo "Tearing down ros2_robotica..."
        down
elif [[ $1 = "rebuild" ]]; then
        echo  "Rebuilding ros2_robotica..."
        rebuild
elif [[ $1 = "build" ]]; then
        echo  "Building ros2_robotica..."
        build
elif [[ $1 = "open" ]]; then
        echo "Opening terminal..."
        open
else
        usage
fi

