# Resumen Teórico — TP Final: Robot Omnidireccional con Mecanum Wheels

## 1. ROS2 y sus componentes

### 1.1 Qué es ROS2

ROS2 (Robot Operating System 2) es un framework de middleware para sistemas robóticos. No es un sistema operativo en el sentido tradicional sino una capa de comunicación y herramientas que permite construir sistemas robóticos como un conjunto de procesos independientes que intercambian datos entre sí.

La unidad básica de ejecución es el **nodo**: un proceso que realiza una tarea específica (leer sensores, calcular odometría, controlar motores). Los nodos se comunican mediante **tópicos** y comparten información espacial mediante el sistema **TF2**.

### 1.2 Nodos

Un nodo es un proceso independiente con una responsabilidad bien definida. En este trabajo hay cuatro nodos propios:

| Nodo | Responsabilidad |
|------|-----------------|
| `omni_odometry_node` | Cinemática inversa y directa; publica odometría |
| `trajectory_follower_node` | Genera waypoints y ejecuta el controlador |
| `laser_detector_node` | Detecta postes en el scan del LiDAR |
| `ekf_node` | Fusiona odometría y LiDAR mediante EKF |

CoppeliaSim actúa como un nodo externo que publica los datos del simulador (encoders, laser, ground truth) y recibe los comandos de velocidad de cada rueda.

### 1.3 Tópicos

Un tópico es un canal de comunicación con nombre y tipo de mensaje fijo. Los nodos se comunican de forma anónima: el publicador no sabe quién lee, el suscriptor no sabe quién escribe. La comunicación es asincrónica y basada en el modelo *publish-subscribe*.

Cada mensaje tiene un tipo definido que determina su estructura. Los tipos usados en este trabajo:

| Tipo | Contenido |
|------|-----------|
| `geometry_msgs/Twist` | Velocidad lineal (x, y, z) + angular (x, y, z) |
| `std_msgs/Float64` | Un número de punto flotante |
| `robmovil_msgs/MultiEncoderTicks` | Ticks acumulados de los 4 encoders |
| `nav_msgs/Odometry` | Pose + velocidad del robot con covarianza |
| `sensor_msgs/LaserScan` | Barrido completo del LiDAR (distancias por ángulo) |
| `robmovil_msgs/LandmarkArray` | Lista de landmarks detectados (range, bearing) |
| `geometry_msgs/PoseArray` | Lista de poses (posición + orientación) |

### 1.4 El sistema TF2 y los frames de referencia

TF2 es el subsistema de ROS2 que gestiona transformaciones entre sistemas de coordenadas. En robótica móvil coexisten múltiples sistemas de referencia simultáneamente: el del mapa global, el del robot, el de cada sensor. TF2 permite expresar cualquier punto o vector en cualquiera de estos sistemas sin hacer los cálculos manualmente.

#### Qué es un frame

Un frame es simplemente un nombre que identifica un sistema de ejes (x, y, z) con un origen en el espacio. No es un nodo ni un tópico: es una etiqueta. Los frames usados en este trabajo son:

| Frame | Qué representa |
|-------|----------------|
| `map` | Sistema de coordenadas global y fijo del mapa |
| `odom` | Origen de la odometría (donde arrancó el robot) |
| `base_link` | Centro de masa del robot; se mueve con él |
| `base_link_gt` | Pose real del robot según la simulación (ground truth) |
| `base_link_ekf` | Pose estimada por el EKF (fusión odometría + LiDAR) |
| `front_laser` | Posición del sensor LiDAR montado en el chasis |

#### Cómo se conectan: el árbol TF

Los frames se organizan en un árbol. Cada rama del árbol es una transformación entre un frame padre y un frame hijo, publicada continuamente en el tópico `/tf`:

```
map ──(sim)──► odom ──(omni_odometry_node)──► base_link
 │                └──(sim)──► base_link_gt
 └──(ekf_node)──────────────────────────────► base_link_ekf
                                  base_link ──(sim)──► front_laser
```

Cada flecha representa un `TransformStamped`: un mensaje que dice "el frame hijo está a esta traslación y rotación respecto al frame padre, en este instante de tiempo". El tópico `/tf` es el canal donde todos estos mensajes fluyen continuamente.

La biblioteca `tf2_ros` acumula estos mensajes en un buffer local. Cuando un nodo necesita saber la relación entre dos frames cualesquiera, llama a `lookupTransform("A", "B")` y el buffer reconstruye la cadena de transformaciones que los conecta. Por ejemplo, `map → base_link` no existe como transform directo: el buffer la obtiene componiendo `map→odom` (publicada por la simulación) con `odom→base_link` (publicada por `omni_odometry_node`).

#### Para qué se usa cada transform en este trabajo

| Transform | Lo publica | Lo usa | Para qué |
|-----------|-----------|--------|----------|
| `map → odom` | CoppeliaSim | TF Buffer (automático) | Conecta el árbol odométrico al frame del mapa |
| `odom → base_link` | `omni_odometry_node` | `trajectory_follower_node` | Pose del robot según encoders |
| `map → base_link_ekf` | `ekf_node` | `trajectory_follower_node` | Pose refinada para el controlador (Ej. 3.5) |
| `odom → base_link_gt` | CoppeliaSim | RViz | Ground truth para validar odometría y EKF |
| `base_link → front_laser` | CoppeliaSim | `ekf_node` (implícito) | Posición fija del sensor en el chasis |

---

## 2. Modelo Cinemático del Robot Omnidireccional

### 1.1 Descripción del robot

El robot utiliza cuatro ruedas Mecanum, cada una con rodillos montados a 45° respecto al eje de la rueda. Esta configuración permite movimiento omnidireccional: el robot puede desplazarse en cualquier dirección en el plano sin necesidad de rotar previamente.

Parámetros geométricos del modelo:
- Radio de rueda: `r = 0.05 m`
- Semi-distancia longitudinal entre ejes: `L_x = 0.175 m`
- Semi-distancia lateral entre ruedas: `L_y = 0.175 m`

Orden de ruedas: `[FL, FR, RL, RR]` (front-left, front-right, rear-left, rear-right).

### 1.2 Cinemática inversa (Twist → velocidades de rueda)

Dado un comando de velocidad `(vx, vy, ω)` expresado en el frame del robot, las velocidades angulares de cada rueda se obtienen invirtiendo la matriz de Mecanum (Ec. 20 del enunciado):

```
w_FL = (1/r) * ( vx - vy - (Lx + Ly) * ω )
w_FR = (1/r) * ( vx + vy + (Lx + Ly) * ω )
w_RL = (1/r) * ( vx + vy - (Lx + Ly) * ω )
w_RR = (1/r) * ( vx - vy + (Lx + Ly) * ω )
```

Esta relación es lineal: cada rueda combina los tres grados de libertad del robot. La señal `ω` genera velocidades de signo opuesto en el eje izquierdo vs. derecho, produciendo rotación sin desplazamiento lateral neto.

### 1.3 Cinemática directa (encoders → odometría)

Dado el incremento de ticks de cada encoder `Δticks_i`, se convierte a distancia lineal recorrida por la rueda:

```
d_i = Δticks_i * 2π * r / ticks_por_vuelta
```

Luego, por las ecuaciones 22-24 del enunciado (pseudoinversa de la matriz Mecanum):

```
Δx_robot  = ( d_FL + d_FR + d_RL + d_RR ) / 4
Δy_robot  = (-d_FL + d_FR + d_RL - d_RR ) / 4
Δθ        = (-d_FL + d_FR - d_RL + d_RR ) / (4 * (Lx + Ly))
```

Las componentes `Δx_robot` y `Δy_robot` están expresadas en el frame del robot. Para integrar en el frame global (odom), se aplica una rotación por el ángulo actual `θ`:

```
x   += Δx_robot * cos(θ) - Δy_robot * sin(θ)
y   += Δx_robot * sin(θ) + Δy_robot * cos(θ)
θ   += Δθ
```

Esta integración es exacta para desplazamientos infinitesimales y se aproxima bien a frecuencias de encoder altas. El error acumula con el tiempo (drift odométrico), motivando el uso del EKF en el Ejercicio 3.

---

## 2. Control a Lazo Cerrado y Seguimiento de Trayectoria

### 2.1 Generación de trayectoria

La trayectoria consiste en un cuadrado de 2 m de lado, recorrido en sentido antihorario. Las esquinas, en orden de recorrido, son:

```
(2, -2) → (2, 2) → (-2, 2) → (-2, -2) → (vuelta a inicio)
```

Cada lado se subdivide en 20 waypoints intermedios, totalizando 80 poses objetivo. La orientación de cada waypoint es radialmente alejada del centro: `θ = atan2(y, x)`. Esto mantiene la cara frontal del robot orientada "hacia afuera" del cuadrado durante todo el recorrido.

### 2.2 Controlador Proporcional (P)

Se implementa un controlador P independiente para cada grado de libertad del robot omnidireccional. Dado que el robot es holonómico, los tres DOF se pueden controlar de forma desacoplada.

El error de posición se calcula en frame mapa y luego se transforma al frame del robot (necesario porque `cmd_vel` se interpreta en coordenadas locales):

```
e_x_map   = goal.x - curr_x
e_y_map   = goal.y - curr_y
e_theta   = normalize(goal.θ - curr_θ)

e_x_robot =  cos(curr_θ) * e_x_map + sin(curr_θ) * e_y_map
e_y_robot = -sin(curr_θ) * e_x_map + cos(curr_θ) * e_y_map
```

La ley de control:

```
vx  = Kp_x     * e_x_robot
vy  = Kp_y     * e_y_robot
ω   = Kp_theta * e_theta
```

Parámetros por defecto: `Kp_x = 1.0`, `Kp_y = 1.0`, `Kp_theta = 0.5`.

### 2.3 Estrategia Pursuit-Based

El nodo no sigue todos los waypoints secuencialmente con una tolerancia rígida de tiempo. En cambio, avanza al waypoint siguiente cuando la distancia euclídea al actual cae por debajo de `position_tolerance = 0.1 m` **y** el error angular es menor que `angle_tolerance = 0.15 rad`. Esto implementa el patrón *Pursuit-Based*: el robot siempre "persigue" el waypoint inmediato siguiente, avanzando en la lista en cuanto lo alcanza.

El timer de control corre a 20 Hz. En cada ciclo:
1. Lookup TF `map → base_link_ekf` para obtener la pose actual.
2. Cálculo del error y publicación de `cmd_vel`.
3. Verificación de la condición de avance.

---

## 3. Localización con Filtro de Kalman Extendido (EKF)

### 3.1 Detección de postes con LiDAR

El nodo `laser_detector_node` procesa el scan del LiDAR frontal para extraer postes como landmarks. El algoritmo tiene tres pasos:

1. **Conversión polar → cartesiana**: cada rayo válido `(r, angle)` se convierte a punto `(x, y)` en frame `base_link`.
2. **Clusterización por distancia**: se agrupan puntos consecutivos cuya separación es menor a `CLUSTER_DIST = 0.1 m`. Un salto mayor indica que dos objetos distintos están en la escena.
3. **Filtro por ancho**: se conservan solo los clusters cuyo diámetro estimado (distancia entre primer y último punto) es compatible con un poste de `0.1 m` de diámetro (tolerancia `±0.08 m`).

El centroide de cada cluster aceptado se publica como `Landmark(range, bearing)` en `/landmarks` con frame `base_link`.

### 3.2 Modelo EKF

#### Estado y control

```
μ = [x, y, θ]ᵀ          (pose del robot en frame mapa)
u = [vx, vy, ω]ᵀ         (velocidades en frame robot, provenientes de odometría)
z = [range, bearing]ᵀ    (medición LiDAR por poste detectado)
```

#### Etapa de predicción

Se integra el modelo de movimiento con un paso Euler:

```
f(μ, u) = μ + [vx*cos(θ) - vy*sin(θ),
               vx*sin(θ) + vy*cos(θ),
               ω                       ] * Δt
```

El Jacobiano del modelo respecto al estado es:

```
F = I₃ + Δt * [ 0,  0,  -vx*sin(θ) - vy*cos(θ) ]
              [ 0,  0,   vx*cos(θ) - vy*sin(θ)  ]
              [ 0,  0,   0                        ]
```

La covarianza se propaga como: `Σ = F * Σ * Fᵀ + Q`

#### Etapa de corrección

Para cada landmark detectado, se busca el poste correspondiente en el mapa conocido (*data association*). El modelo de sensado predice la observación esperada desde la pose actual:

```
h(μ) = [ sqrt((mx - px)² + (my - py)²),
          atan2(my - py, mx - px) - θ   ]
```

El Jacobiano de la observación:

```
H = [ -dx/d,  -dy/d,   0 ]
    [  dy/d², -dx/d², -1  ]

donde dx = mx - px,  dy = my - py,  d = sqrt(dx² + dy²)
```

La actualización del EKF estándar:

```
S = H * Σ * Hᵀ + R
K = Σ * Hᵀ * S⁻¹                  (ganancia de Kalman)
μ = μ + K * (z - h(μ))             (corrección del estado)
Σ = (I - K * H) * Σ                (reducción de incertidumbre)
```

#### Matrices de covarianza

| Matriz | Valor | Interpretación |
|--------|-------|----------------|
| Q (proceso) | `diag(0.05², 0.05², (2°)²)` | Incertidumbre en la predicción odométrica |
| R (sensado) | `diag(0.1², (5°)²)` | Incertidumbre en la medición LiDAR |
| Σ₀ (inicial) | `0.01 * I₃` | Certeza alta en la pose inicial |

La elección `R < Q` (en términos de magnitud relativa) hace que la corrección por LiDAR tenga mayor peso que la predicción odométrica, lo que permite compensar el drift acumulado.

### 3.3 Data association

Para cada observación `(range, bearing)` se busca el poste del mapa que minimiza la distancia:

```
dist = |z_range - exp_range| + |z_bearing - exp_bearing|
```

Si la distancia mínima supera el umbral de 2.0 m (en espacio de observación), el landmark se descarta para evitar asociaciones incorrectas.

### 3.4 Integración con el control

El nodo `trajectory_follower_node` usa `map → base_link_ekf` como feedback en lugar de `map → base_link`. Al reemplazar la odometría pura por la estimación EKF, el error de pose acumulado se corrige periódicamente cada vez que el robot observa un poste conocido, permitiendo un seguimiento de trayectoria más preciso a largo plazo.

---

## 4. Arquitectura del Sistema

### Nodos

| Nodo | Responsabilidad |
|------|-----------------|
| `omni_odometry_node` | Cinemática inversa + integración de encoders + TF `odom→base_link` |
| `trajectory_follower_node` | Generación de waypoints + controlador P + Pursuit-Based |
| `laser_detector_node` | Detección de postes desde LaserScan |
| `ekf_node` | Fusión odometría + LiDAR + TF `map→base_link_ekf` |

### Flujo de datos

```
CoppeliaSim
  ├─ /robot/encoders ──► omni_odometry_node ──► /robot/odometry
  │                              └──────────────► TF: odom→base_link
  ├─ /robot/front_laser/scan ──► laser_detector_node ──► /landmarks
  └─ TF: map→odom  (fijo)
         TF: base_link→front_laser

/robot/odometry + /landmarks + /posts
  └──► ekf_node ──► TF: map→base_link_ekf

TF: map→base_link_ekf
  └──► trajectory_follower_node ──► /robot/cmd_vel ──► omni_odometry_node ──► ruedas
```

### Tópicos principales

| Tópico | Tipo | Dirección |
|--------|------|-----------|
| `/robot/cmd_vel` | `geometry_msgs/Twist` | follower → odometry |
| `/robot/encoders` | `robmovil_msgs/MultiEncoderTicks` | sim → odometry |
| `/robot/{fl,fr,rl,rr}_wheel/cmd_vel` | `std_msgs/Float64` | odometry → sim |
| `/robot/odometry` | `nav_msgs/Odometry` | odometry → EKF |
| `/robot/front_laser/scan` | `sensor_msgs/LaserScan` | sim → detector |
| `/landmarks` | `robmovil_msgs/LandmarkArray` | detector → EKF |
| `/posts` | `geometry_msgs/PoseArray` | sim → EKF (mapa) |
| `/trajectory_waypoints` | `geometry_msgs/PoseArray` | follower → RViz |
