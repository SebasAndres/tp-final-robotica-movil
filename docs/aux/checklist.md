# Mapa de implementación — TP Final: Robot Omnidireccional

Referencia cruzada entre cada inciso del enunciado y el código que lo resuelve.

---

## Ejercicio 1 — Modelo Cinemático

### 1.a — Cinemática inversa: `/robot/cmd_vel` → velocidades de rueda

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Subscriber a `/robot/cmd_vel` (Twist) | `omni_odometry.cpp:14-19` |
| Callback que calcula velocidades por rueda (Ec. 20) | `omni_odometry.cpp:49-77` |
| Publishers a `/robot/{front_left,front_right,rear_left,rear_right}_wheel/cmd_vel` | `omni_odometry.cpp:28-36` |

### 1.b — Cinemática directa: encoders → odometría

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Subscriber a `/robot/encoders` (MultiEncoderTicks) | `omni_odometry.cpp:20-26` |
| Callback: delta ticks → arc_length por rueda | `omni_odometry.cpp:80-108` |
| Cinemática directa Mecanum (Ec. 22-24): Δx, Δy, Δθ en frame robot | `omni_odometry.cpp:109-111` |
| Integración en frame global (odom) | `omni_odometry.cpp:114-117` |

### 1.c — Publicación odométrica

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Publicación `nav_msgs/Odometry` en `/robot/odometry` | `omni_odometry.cpp:120-137` |
| TF broadcaster `odom → base_link` en `/tf` | `omni_odometry.cpp:140-150` |

> **Nota:** `map → odom` lo publica la simulación (`/vrep_ros_interface`), no este nodo.

---

## Ejercicio 2 — Control a lazo cerrado y seguimiento de trayectoria

### 2.1 — Controlador Proporcional (P)

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Parámetros Kp_x, Kp_y, Kp_theta (declarados con defaults) | `trajectory_follower.cpp:14-19` |
| Error de posición en frame mapa → transformado a frame robot | `trajectory_follower.cpp:113-118` |
| Ley de control P: cmd = Kp · error | `trajectory_follower.cpp:121-124` |
| Publicación periódica en `/robot/cmd_vel` | `trajectory_follower.cpp:126` |

### 2.2 — Waypoints de trayectoria cuadrada (Pursuit-Based)

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Generación de trayectoria cuadrada 2m, 20 pts/lado, orientación outward | `trajectory_follower.cpp:62-75` |
| Selección del waypoint activo (Pursuit-Based) | `trajectory_follower.cpp:135-144` |
| Publicación de waypoints como PoseArray en `/trajectory_waypoints` | `trajectory_follower.cpp:37-56` |

### 2.3 — Nodo de seguimiento

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| 2.3.a — Publicación periódica de cmd_vel (timer 20 Hz) | `trajectory_follower.cpp:57-59` |
| 2.3.b — Feedback: lookup TF `map → base_link` | `trajectory_follower.cpp:107-114` |
| 2.3.c — Redefinición de pose objetivo (Pursuit-Based) | `trajectory_follower.cpp:135-144` |

---

## Ejercicio 3 — Seguimiento de trayectoria con EKF

### 3.1 — Nodo detector de postes (LiDAR)

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Subscriber a `/robot/front_laser/scan` | `laser_detector.cpp:14-16` |
| Conversión rayos → puntos cartesianos en frame `base_link` | `laser_detector.cpp:54-65` |
| Clusterización por distancia entre puntos consecutivos | `laser_detector.cpp:67-80` |
| Filtro de clusters por ancho compatible con poste de 0.1m | `laser_detector.cpp:82-95` |
| Publisher `LandmarkArray` en `/landmarks` | `laser_detector.cpp:21-35` |

### 3.2 — Modelado EKF

**Estado:** implementado

| Elemento | Valor |
|----------|-------|
| Estado `x` | `[x, y, θ]` |
| Control `u` | `[vx, vy, ω]` (dim 3, omnidireccional) |
| Medición `z` | `[range, bearing]` por poste detectado |
| Modelo de movimiento `f(x,u)` | Integración Euler en frame global |
| Jacobiano F | `ekf.cpp:on_odometry` — derivada de f respecto a x |
| Modelo de sensado `h(x)` | `[√((mx-px)²+(my-py)²), atan2(my-py,mx-px)−θ]` |
| Jacobiano H | `ekf.cpp:on_landmarks` — derivada de h respecto a x |

### 3.3 — Matrices de covarianza

**Estado:** implementado (`ekf.cpp:18-30`)

| Matriz | Valores | Criterio |
|--------|---------|----------|
| Q (proceso) | diag(0.05², 0.05², (2°)²) | Alta incertidumbre en predicción odométrica |
| R (sensado) | diag(0.1², (5°)²) | Baja incertidumbre en medición LiDAR |

R < Q → la corrección por sensor tiene más peso que la predicción.

### 3.4 — Nodo EKF

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Predicción: `f(x,u)` + propagación de `σ` con F | `ekf.cpp:on_odometry` |
| Corrección: ganancia K, innovación, update de `μ` y `σ` | `ekf.cpp:on_landmarks` |
| Data association: poste más cercano en el mapa | `ekf.cpp:associate()` |
| Publicación TF `map → base_link_ekf` | `ekf.cpp:publish_tf()` |

### 3.5 — Seguimiento con pose EKF

**Estado:** implementado

| Qué | Dónde |
|-----|-------|
| Feedback cambiado a `map → base_link_ekf` | `trajectory_follower.cpp:109` |

> Para volver a odometría pura (sin EKF), cambiar `"base_link_ekf"` por `"base_link"` en esa línea.

---

## Nodos y tópicos del sistema completo

| Nodo | Archivo | Estado |
|------|---------|--------|
| `omni_odometry_node` | `src/omni_odometry.cpp` | implementado |
| `trajectory_follower_node` | `src/trajectory_follower.cpp` | implementado |
| `laser_detector_node` | `src/laser_detector.cpp` | implementado |
| `ekf_node` | `src/ekf.cpp` | implementado |

## Árbol TF

```
map  ──(sim)──►  odom  ──(omni_odometry)──►  base_link
 │                └──(sim)──► base_link_gt
 └──(ekf_node, pendiente)──► base_link_ekf
base_link ──(sim)──► front_laser
```
