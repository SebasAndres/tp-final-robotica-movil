# TP Final — Robot Omnidireccional

Localización y seguimiento de trayectorias de un robot omnidireccional Mecanum en ROS2 + CoppeliaSim, usando odometría, control proporcional y filtro extendido de Kalman (EKF).

## Documentación

- Informe — [HTML](https://sebasandres.github.io/tp-final-robotica-movil/informe.html) | [PDF](docs/informe.pdf)
- [Enunciado](Enunciado.pdf)
- [Paper — Taheri et al. (2015)](Paper.pdf)

## Ejecutar

Lanza CoppeliaSim, RViz2 y los cuatro nodos en paneles de Terminator. Compila el paquete automáticamente.

```bash
# desde el host, con el contenedor corriendo
KP_X=1.0 KP_Y=1.0 KP_THETA=1.5 POS_TOL=0.1 ANG_TOL=0.15 \
  ./scripts/launch_terminator.sh
```

## Compilar el informe

Requiere `pandoc` y `google-chrome` en el host.

```bash
cd docs && ./build.sh
```

## Pendientes

### Sección 1 — Modelo cinemático

- [ ] **Exp. 1.1 — Validación cinemática inversa:** enviar consignas de velocidad constante para cada GDL por separado ($v_x$ puro, $v_y$ puro, $\omega_z$ puro) y graficar las velocidades angulares comandadas vs. las reportadas por el simulador.
- [ ] **Exp. 1.2 — Validación de odometría:** recorrer trayectorias simples (traslación X, traslación Y, rotación en el lugar) y comparar la pose estimada (`odom→base_link`) contra el ground truth (`odom→base_link_gt`).

### Sección 2 — Control a lazo cerrado

- [ ] **Exp. 2.1 — Selección de $\varepsilon_\text{pos},\, \varepsilon_\theta$:** experimentar con distintos valores de tolerancia y justificar la elección final.
- [ ] **Exp. 2.2 — Selección de $k_{p,*}$:** sintonizar las ganancias del controlador P y justificar los valores elegidos.
- [ ] Agregar imagen de la trayectoria cuadrada en RViz2 (`docs/assets/cuadrado_rviz`).

### Sección 3 — Localización EKF

- [ ] **Justificar $Q$:** explicar por qué se eligieron los valores `diag(0.05², 0.05², (2°)²)` para el ruido de proceso.
- [ ] **Justificar $R$:** explicar por qué se eligieron los valores `diag(0.1², (5°)²)` para el ruido del sensor LiDAR.
- [ ] **Exp. 3.1 — Matplotlib:** comparar trayectoria estimada por EKF vs. ground truth para velocidades constantes.
- [ ] **Exp. 3.2 — RViz2:** visualizar el elipsoide de covarianza durante el recorrido y analizar cómo evoluciona al detectar postes.
