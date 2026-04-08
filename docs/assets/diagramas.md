# Diagramas del sistema

## 1. Nodos y tópicos

```mermaid
flowchart LR

    SIM([CoppeliaSim])

    subgraph omni["omni_odometry_node"]
        OO["Cinemática inversa\n+ integración encoders"]
    end

    subgraph follower["trajectory_follower_node"]
        TF["Controlador P\n+ Pursuit-Based"]
    end

    subgraph detector["laser_detector_node"]
        LD["Clusterización\nLiDAR"]
    end

    subgraph ekf["ekf_node"]
        EK["Predicción\n+ Corrección EKF"]
    end

    RVIZ([RViz])

    %% Simulación → omni_odometry
    SIM -->|"/robot/encoders\nMultiEncoderTicks"| OO

    %% omni_odometry → simulación (ruedas)
    OO -->|"/robot/front_left_wheel/cmd_vel\nFloat64"| SIM
    OO -->|"/robot/front_right_wheel/cmd_vel\nFloat64"| SIM
    OO -->|"/robot/rear_left_wheel/cmd_vel\nFloat64"| SIM
    OO -->|"/robot/rear_right_wheel/cmd_vel\nFloat64"| SIM

    %% omni_odometry → EKF
    OO -->|"/robot/odometry\nOdometry"| EK

    %% trajectory_follower → omni_odometry
    TF -->|"/robot/cmd_vel\nTwist"| OO

    %% Simulación → laser_detector
    SIM -->|"/robot/front_laser/scan\nLaserScan"| LD

    %% laser_detector → EKF
    LD -->|"/landmarks\nLandmarkArray"| EK

    %% Simulación → EKF (mapa de postes)
    SIM -->|"/posts\nPoseArray"| EK

    %% Debug: follower → RViz
    TF -->|"/trajectory_waypoints\nPoseArray"| RVIZ
```

---

## 2. Árbol de transformaciones TF

```mermaid
flowchart TD

    MAP(["map\n(frame global del mapa)"])
    ODOM(["odom\n(frame odométrico)"])
    BL(["base_link\n(centro del robot)"])
    BL_GT(["base_link_gt\n(ground truth sim)"])
    BL_EKF(["base_link_ekf\n(estimación EKF)"])
    FL(["front_laser\n(sensor LiDAR)"])

    MAP -->|"publicado por: CoppeliaSim\nusado para: anclar odometría al mapa\ntipo: casi estático (deriva mínima)"| ODOM
    ODOM -->|"publicado por: omni_odometry_node\nusado para: pose del robot por encoders\nse actualiza: cada mensaje de encoders"| BL
    ODOM -->|"publicado por: CoppeliaSim\nusado para: validación / comparación"| BL_GT
    MAP -->|"publicado por: ekf_node\nusado para: pose refinada (fusión LiDAR)\nse actualiza: cada mensaje de odometría"| BL_EKF
    BL -->|"publicado por: CoppeliaSim\nusado para: proyectar scan al frame mapa\ntipo: estático (sensor fijo al chasis)"| FL
```

---

## 3. Quién lee cada transformación y para qué

| Transform | Lo publica | Lo lee | Para qué |
|-----------|-----------|--------|----------|
| `map → odom` | CoppeliaSim | TF Buffer (automático) | Eslabón que conecta los dos árboles |
| `odom → base_link` | `omni_odometry_node` | `trajectory_follower_node` (indirectamente vía `map→base_link`) | Pose odométrica del robot |
| `map → base_link_ekf` | `ekf_node` | `trajectory_follower_node` (feedback de control) | Pose corregida con LiDAR para el controlador P |
| `odom → base_link_gt` | CoppeliaSim | RViz (solo visualización) | Ground truth para validar odometría y EKF |
| `base_link → front_laser` | CoppeliaSim | `ekf_node` (implícito en modelo h(x)) | Posición del sensor en el chasis |

### Cómo se compone `map → base_link`

El Buffer de TF2 nunca recibe ese transform directamente.
Lo reconstruye concatenando dos transforms que sí existen:

```
map → base_link  =  (map → odom)  ∘  (odom → base_link)
                      [sim]              [omni_odometry_node]
```

Esto es lo que `lookupTransform("map", "base_link", ...)` devuelve internamente,
y es lo que `trajectory_follower_node` usaría si el feedback fuera odometría pura.

Con EKF activo, el follower pide `map → base_link_ekf` directamente —
no hay composición porque ese transform va directo de `map`.
