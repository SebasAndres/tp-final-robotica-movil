#!/usr/bin/env bash
# Abre Terminator con 6 paneles ejecutando los nodos de modelo_omnidireccional.
# Ejecutar desde el HOST. El contenedor ros2_robotica debe estar corriendo.
# El paquete debe estar compilado en el contenedor antes de lanzar los nodos:
#   docker exec -it ros2_robotica bash -c "cd /root/ros2_ws && colcon build --packages-select modelo_omnidireccional"

set -euo pipefail

CONTAINER="ros2_robotica"
SETUP="source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash"
COPPELIA_SH="/root/scripts/coppeliaSim.sh"
RVIZ_CFG="/root/coppeliaSim/tpfinal.rviz"

KP_X="${KP_X:-1.0}"
KP_Y="${KP_Y:-1.0}"
KP_THETA="${KP_THETA:-1.5}"
POS_TOL="${POS_TOL:-0.1}"
ANG_TOL="${ANG_TOL:-0.15}"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo "Error: el contenedor '${CONTAINER}' no está corriendo."
    echo "Inicialo con: ./start-docker.sh start"
    exit 1
fi

echo "Compilando paquetes en el contenedor..."
docker exec "${CONTAINER}" bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /root/ros2_ws &&
    colcon build --packages-select robmovil_msgs --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 &&
    colcon build --packages-select modelo_omnidireccional --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1
" || { echo "Error: falló la compilación. Revisá los logs arriba."; exit 1; }

CFG=$(mktemp /tmp/terminator_omni_XXXX.cfg)

# Layout 6 paneles (3 filas x 2 columnas):
#   ┌──────────────┬──────────────┐
#   │  coppeliaSim │  rviz2       │  ← hpaned_top
#   ├──────────────┼──────────────┤
#   │  odometry    │  laser       │  ← hpaned_bot: vpaned_left | vpaned_right
#   ├──────────────┼──────────────┤
#   │  ekf         │  trajectory  │
#   └──────────────┴──────────────┘
#
# Árbol: window0 → vpaned0 → hpaned_top (coppelia | rviz2)         [order=0]
#                           → hpaned_bot (vpaned_left | vpaned_right) [order=1]
#                                vpaned_left  → odometry [order=0] + ekf [order=1]
#                                vpaned_right → laser    [order=0] + trajectory [order=1]
# Evita VPaned→VPaned (bug de Terminator). Usa VPaned→(HPaned,HPaned) igual que el
# layout 2x2 que funciona, con HPaned→(VPaned,VPaned) para la sección de nodos.

cat > "$CFG" <<EOF
[global_config]
[keybindings]
[profiles]
  [[default]]
[layouts]
  [[omni]]
    [[[window0]]]
      type = Window
      parent = ""
      title = ROS2 - modelo_omnidireccional
    [[[vpaned0]]]
      type = VPaned
      parent = window0
    [[[hpaned_top]]]
      type = HPaned
      parent = vpaned0
      order = 0
    [[[hpaned_bot]]]
      type = HPaned
      parent = vpaned0
      order = 1
    [[[vpaned_left]]]
      type = VPaned
      parent = hpaned_bot
      order = 0
    [[[vpaned_right]]]
      type = VPaned
      parent = hpaned_bot
      order = 1
    [[[coppelia]]]
      type = Terminal
      parent = hpaned_top
      order = 0
      title = coppeliaSim
      command = docker exec -it ${CONTAINER} bash -c "${COPPELIA_SH}; exec bash"
    [[[rviz]]]
      type = Terminal
      parent = hpaned_top
      order = 1
      title = rviz2
      command = docker exec -it ${CONTAINER} bash -c "${SETUP} && rviz2 -d ${RVIZ_CFG}; exec bash"
    [[[odometry]]]
      type = Terminal
      parent = vpaned_left
      order = 0
      title = omni_odometry_node
      command = docker exec -it ${CONTAINER} bash -c "${SETUP} && ros2 run modelo_omnidireccional omni_odometry_node; exec bash"
    [[[ekf]]]
      type = Terminal
      parent = vpaned_left
      order = 1
      title = ekf_node
      command = docker exec -it ${CONTAINER} bash -c "${SETUP} && ros2 run modelo_omnidireccional ekf_node; exec bash"
    [[[laser]]]
      type = Terminal
      parent = vpaned_right
      order = 0
      title = laser_detector_node
      command = docker exec -it ${CONTAINER} bash -c "${SETUP} && ros2 run modelo_omnidireccional laser_detector_node; exec bash"
    [[[trajectory]]]
      type = Terminal
      parent = vpaned_right
      order = 1
      title = trajectory_follower_node
      command = docker exec -it ${CONTAINER} bash -c "${SETUP} && ros2 run modelo_omnidireccional trajectory_follower_node --ros-args -p kp_x:=${KP_X} -p kp_y:=${KP_Y} -p kp_theta:=${KP_THETA} -p position_tolerance:=${POS_TOL} -p angle_tolerance:=${ANG_TOL}; exec bash"
[plugins]
EOF

# Terminator usa single-instance via DBus: si hay una instancia corriendo,
# ignora --config y usa el layout cacheado. Hay que cerrar las otras instancias,
# pero no la actual (la que ejecuta este script).
MY_TERM_PID=""
pid=$$
while [ "$pid" -ne 1 ]; do
    pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$pid" ] && break
    cmd=$(ps -o comm= -p "$pid" 2>/dev/null || true)
    if [ "$cmd" = "terminator" ]; then
        MY_TERM_PID="$pid"
        break
    fi
done

OTHER_TERMS=$(pgrep -x terminator | grep -v "^${MY_TERM_PID}$" || true)
if [ -n "$OTHER_TERMS" ]; then
    echo "Cerrando instancias previas de Terminator..."
    echo "$OTHER_TERMS" | xargs kill 2>/dev/null || true
    sleep 1
fi

terminator --config "$CFG" --layout omni &
