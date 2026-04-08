# Usar la imagen oficial de ROS2 Humble
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# --------------------
# Instalación de dependencias generales
# --------------------
RUN apt-get update && apt-get install -y \
    gedit nano wget evince vim \
    ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3* ros-humble-urdf-tutorial \
    ros-humble-rqt-tf-tree python3-pip tree unzip libglu1-mesa libxi6 libxmu6 \
    libglu1 libgl1-mesa-glx libxcb-xinerama0 libsdl1.2-dev ros-humble-imu-tools\
    && rm -rf /var/lib/apt/lists/*

# --------------------
# Instalar CoppeliaSim
# --------------------
# Cambia la URL si quieres otra versión (aquí ejemplo con 4.7.0)
#ENV COPPELIASIM_VERSION=4.7.0

RUN wget -q https://downloads.coppeliarobotics.com/V4_7_0_rev2/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04.tar.xz \
    && tar -xf CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04.tar.xz -C /opt \
    && rm CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04.tar.xz

# Añadir CoppeliaSim al PATH
ENV PATH="/opt/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04:${PATH}"

# --------------------
# Instalar bridge ROS2 <-> CoppeliaSim
# --------------------
# El "ros2-coppeliasim" es mantenido por CoppeliaRobotics en GitHub, se instala con pip
RUN pip3 install pyzmq cbor

# Clonamos y compilamos el plugin ROS2 para CoppeliaSim
WORKDIR /root/ros2_ws/src
RUN apt-get update && apt-get install -y git && apt-get install -y \
    build-essential cmake libx11-dev libgl1-mesa-dev libglu1-mesa-dev xsltproc \
    && pip3 install xmlschema
RUN git clone --depth 1 https://github.com/CoppeliaRobotics/simROS2.git sim_ros2_interface \
    && cd sim_ros2_interface && git fetch --tags && git checkout coppeliasim-v4.7.0-rev2
WORKDIR /root/ros2_ws

# --------------------
# Copiar robmovil_msgs al workspace
# --------------------
COPY volume/robmovil_msgs /root/ros2_ws/src/robmovil_msgs

# --------------------
# Registrar tipos de mensaje en sim_ros2_interface (interfaces.txt)
# --------------------
RUN { \
      echo "geometry_msgs/msg/PoseArray"; \
      echo "geometry_msgs/msg/PoseStamped"; \
      echo "geometry_msgs/msg/PoseWithCovarianceStamped"; \
      echo "geometry_msgs/msg/TransformStamped"; \
      echo "sensor_msgs/msg/LaserScan"; \
      echo "sensor_msgs/msg/Imu"; \
      echo "sensor_msgs/msg/Joy"; \
      echo "nav_msgs/msg/Odometry"; \
      echo "nav_msgs/msg/Path"; \
      echo "std_msgs/msg/Float64"; \
      echo "std_msgs/msg/Int32"; \
      echo "trajectory_msgs/msg/JointTrajectoryPoint"; \
      echo "trajectory_msgs/msg/JointTrajectory"; \
      echo "rosgraph_msgs/msg/Clock"; \
      echo "robmovil_msgs/msg/MultiEncoderTicks"; \
      echo "robmovil_msgs/msg/EncoderTicks"; \
      echo "robmovil_msgs/msg/Trajectory"; \
      echo "robmovil_msgs/msg/TrajectoryPoint"; \
      echo "robmovil_msgs/msg/LandmarkArray"; \
      echo "robmovil_msgs/msg/Landmark"; \
    } >> /root/ros2_ws/src/sim_ros2_interface/meta/interfaces.txt

# --------------------
# Agregar dependencia robmovil_msgs a sim_ros2_interface
# --------------------
RUN sed -i 's|</package>|  <depend>robmovil_msgs</depend>\n</package>|' \
      /root/ros2_ws/src/sim_ros2_interface/package.xml

RUN sed -i '/find_package(ament_cmake REQUIRED)/a find_package(robmovil_msgs REQUIRED)' \
      /root/ros2_ws/src/sim_ros2_interface/CMakeLists.txt \
    && sed -i '/ament_target_dependencies/,/^)/{s/^)/  robmovil_msgs\n)/}' \
      /root/ros2_ws/src/sim_ros2_interface/CMakeLists.txt

# --------------------
# Compilar workspace (robmovil_msgs primero, luego sim_ros2_interface)
# --------------------
ENV COPPELIASIM_ROOT_DIR="/opt/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCOPPELIASIM_ROOT_DIR=$COPPELIASIM_ROOT_DIR" \
    && touch /root/ros2_ws/src/robmovil_msgs/COLCON_IGNORE

# --------------------
# Configurar entorno
# --------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc \
    && echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=12" >> ~/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models/:/root/catkin_ws/volumen/models" >> ~/.bashrc \
    && echo "export PATH=/opt/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu20_04:\$PATH" >> ~/.bashrc 

# --------------------
# Instalar visual code
# --------------------
WORKDIR /root
RUN apt-get update && apt-get install -y wget gpg apt-transport-https software-properties-common && \
	wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg && \
	install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/ && \
	sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' && \
	apt-get update && apt-get install -y code


CMD ["bash"]
