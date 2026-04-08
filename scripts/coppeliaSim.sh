#!/usr/bin/env bash
# Lanza CoppeliaSim con la escena del TP Final.
# Ejecutar DENTRO del contenedor Docker.
# Usa el script oficial para setear LD_LIBRARY_PATH correctamente.

COPPELIA_SH="/opt/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04/coppeliaSim.sh"
SCENE="/root/coppeliaSim/omni_ekf.ttt"

# Source ROS2 so the simROS2 plugin can find librosidl_typesupport_cpp.so
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

exec "$COPPELIA_SH" "$SCENE"
