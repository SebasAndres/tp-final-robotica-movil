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
