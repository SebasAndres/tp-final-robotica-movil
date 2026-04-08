# Filtro de Kalman y Filtro de Kalman Extendido (EKF)

## 1. Problema que resuelve

En robótica móvil, el robot necesita conocer su pose (posición y orientación) en todo momento. Hay dos fuentes de información disponibles:

- **Modelo de movimiento (odometría):** a partir de lo que se le ordenó hacer y de los encoders, se puede predecir dónde debería estar el robot. Esta predicción acumula error con el tiempo (*drift*).
- **Sensores externos (LiDAR, GPS, cámara):** permiten observar el entorno y corregir la estimación, pero tienen ruido en cada medición individual.

El Filtro de Kalman es el algoritmo óptimo para combinar ambas fuentes de información cuando el sistema es lineal y los ruidos son gaussianos. Para sistemas no lineales (como la cinemática de un robot móvil) se usa la variante **EKF (Extended Kalman Filter)**, que linealiza el sistema en cada paso.

---

## 2. Representación probabilística de la pose

En lugar de mantener una pose exacta, el EKF mantiene una **distribución de probabilidad** sobre la pose. Bajo la suposición de ruido gaussiano, esta distribución queda completamente descripta por dos parámetros:

- **μ (media):** la estimación más probable de la pose.
- **Σ (covarianza):** cuánta incertidumbre hay sobre esa estimación. Una covarianza grande significa que el robot no sabe bien dónde está.

En este trabajo:

```
μ = [x, y, θ]ᵀ    (pose estimada en frame mapa)

Σ = | σ_xx  σ_xy  σ_xθ |    (matriz 3×3 de incertidumbre)
    | σ_yx  σ_yy  σ_yθ |
    | σ_θx  σ_θy  σ_θθ |
```

Los elementos diagonales de Σ son las varianzas de cada componente. Los elementos fuera de la diagonal capturan correlaciones: por ejemplo, si el robot giró hacia la derecha, hay correlación entre el error en x y el error en y.

---

## 3. El ciclo del EKF

El EKF alterna dos etapas en cada paso de tiempo:

```
┌──────────────────────────────────────────────────────┐
│  [estado anterior: μ, Σ]                             │
│         │                                            │
│         ▼                                            │
│   PREDICCIÓN (con odometría)                         │
│   → μ̄, Σ̄  (creencia a priori, más incierta)         │
│         │                                            │
│         ▼                                            │
│   CORRECCIÓN (con sensor)                            │
│   → μ, Σ  (creencia a posteriori, más certera)       │
│         │                                            │
│         └──────────────────► [siguiente paso]        │
└──────────────────────────────────────────────────────┘
```

---

## 4. Etapa de predicción

### Intuición

"Sé dónde estaba. Aplico el modelo de movimiento. Ahora sé dónde *debería* estar, pero con más incertidumbre que antes porque el modelo no es perfecto."

### Modelo de movimiento

El modelo `f(μ, u)` describe cómo evoluciona la pose dado un control `u = [vx, vy, ω]`:

```
f(μ, u) = | x + (vx·cos(θ) - vy·sin(θ))·Δt |
           | y + (vx·sin(θ) + vy·cos(θ))·Δt |
           | θ + ω·Δt                         |
```

Este modelo es **no lineal** porque aparece `cos(θ)` y `sin(θ)`, que dependen del estado mismo.

### Propagación de la incertidumbre

Para propagar la covarianza a través de una función no lineal, el EKF linealiza `f` alrededor del estado actual usando su Jacobiano:

```
F = ∂f/∂μ = | 1  0  -vx·sin(θ)·Δt - vy·cos(θ)·Δt |
             | 0  1   vx·cos(θ)·Δt - vy·sin(θ)·Δt |
             | 0  0   1                              |
```

La covarianza se propaga como:

```
Σ̄ = F · Σ · Fᵀ + Q
```

donde **Q** es la matriz de ruido de proceso: cuánta incertidumbre introduce el modelo de movimiento en sí (imperfecciones del actuador, deslizamiento de ruedas, etc.).

Después de la predicción, `Σ̄ ≥ Σ`: la incertidumbre solo puede crecer o mantenerse.

---

## 5. Etapa de corrección

### Intuición

"El sensor me dice algo sobre el entorno. Comparo lo que *observé* con lo que *esperaba* observar desde mi pose predicha. La diferencia me dice hacia dónde debo mover mi estimación."

### Modelo de sensado

El modelo `h(μ)` predice qué debería medir el sensor si el robot estuviera en la pose `μ`. Para un poste conocido en posición `(mx, my)` del mapa:

```
h(μ) = | sqrt((mx - x)² + (my - y)²)    |   = | range esperado   |
        | atan2(my - y, mx - x) - θ      |     | bearing esperado |
```

### Innovación

La innovación es la diferencia entre lo que el sensor midió y lo que el modelo predijo:

```
ν = z - h(μ̄)
```

Si la innovación es cero, el sensor confirma exactamente la predicción y no hay corrección. Cuanto mayor la innovación, mayor el ajuste.

### Jacobiano de la observación

Como `h` también es no lineal, se linealiza:

```
H = ∂h/∂μ = | -dx/d    -dy/d    0  |
             |  dy/d²  -dx/d²  -1  |

donde dx = mx - x,  dy = my - y,  d = sqrt(dx² + dy²)
```

### Ganancia de Kalman

La ganancia K determina cuánto peso se le da al sensor vs. a la predicción:

```
S = H · Σ̄ · Hᵀ + R       (covarianza de la innovación)
K = Σ̄ · Hᵀ · S⁻¹         (ganancia de Kalman, dim 3×2)
```

**R** es la matriz de ruido de medición: cuánta incertidumbre tiene el sensor. Si R es pequeño (sensor preciso), K es grande y el sensor tiene mucho peso. Si R es grande (sensor ruidoso), K es pequeño y se confía más en la predicción.

### Actualización

```
μ  = μ̄ + K · ν            (corrección de la media)
Σ  = (I - K·H) · Σ̄       (reducción de la incertidumbre)
```

Después de la corrección, `Σ ≤ Σ̄`: el sensor siempre reduce la incertidumbre (o la deja igual si K=0).

---

## 6. La ganancia de Kalman como ponderación

K se puede interpretar como un peso que balancea dos fuentes de información:

```
Si R → 0  (sensor perfecto):   K → H⁻¹   → μ = h⁻¹(z)   (creer solo al sensor)
Si R → ∞  (sensor inútil):    K → 0      → μ = μ̄         (ignorar el sensor)
Si Q → 0  (modelo perfecto):  Σ̄ → 0      → K → 0         (creer solo al modelo)
```

En la práctica, K queda en algún punto intermedio. La elección de Q y R es el principal parámetro de diseño del filtro.

---

## 7. Elección de Q y R en este trabajo

| Matriz | Valor | Razonamiento |
|--------|-------|--------------|
| Q | `diag(0.05², 0.05², (2°)²)` | La odometría acumula error: se asume σ=5cm en posición y σ=2° en orientación por paso |
| R | `diag(0.1², (5°)²)` | El LiDAR tiene ruido de ~10cm en rango y ~5° en ángulo por medición |
| Σ₀ | `0.01 · I₃` | El robot arranca en una pose conocida con poca incertidumbre |

La relación `R ≈ 2·Q` en magnitud hace que el sensor tenga algo más de peso que la predicción, lo que permite corregir el drift acumulado. Si Q fuera mucho menor que R, el filtro confiaría casi exclusivamente en la odometría y no se beneficiaría del LiDAR.

---

## 8. Data association

Antes de aplicar la corrección, es necesario saber **qué poste del mapa corresponde a cada detección**. Este problema se llama *data association*.

En este trabajo se usa el método del vecino más cercano en espacio de observación: para cada landmark detectado `(range, bearing)`, se calcula la observación esperada desde `μ̄` para cada poste del mapa conocido, y se elige el que minimiza:

```
dist = |z_range - h_range(μ̄, poste_i)| + |z_bearing - h_bearing(μ̄, poste_i)|
```

Si la distancia mínima supera un umbral (2.0 en este caso), el landmark se descarta: podría ser un objeto no mapeado o una detección falsa.

---

## 9. Diferencia entre Kalman lineal y EKF

| Aspecto | Kalman lineal (KF) | Kalman extendido (EKF) |
|---------|-------------------|------------------------|
| Modelo de movimiento | `f(x,u) = A·x + B·u` (lineal) | `f(x,u)` cualquier función diferenciable |
| Propagación de Σ | `A·Σ·Aᵀ + Q` | `F·Σ·Fᵀ + Q`, con F=∂f/∂x |
| Modelo de sensado | `h(x) = C·x` (lineal) | `h(x)` cualquier función diferenciable |
| Innovación covarianza | `C·Σ·Cᵀ + R` | `H·Σ·Hᵀ + R`, con H=∂h/∂x |
| Optimalidad | Óptimo para sistemas lineales gaussianos | Aproximación: puede divergir si la no-linealidad es alta |

El EKF es una aproximación de primer orden (linealización local). Para no-linealidades fuertes existen variantes más robustas como el UKF (Unscented Kalman Filter) o el filtro de partículas, a costa de mayor costo computacional.

---

## 10. Resumen del ciclo completo

```
Entrada: odometría (vx, vy, ω, Δt)  →  Predicción:
    μ̄ = f(μ, u)
    Σ̄ = F·Σ·Fᵀ + Q
    publicar TF map→base_link_ekf con μ̄

Entrada: landmarks detectados (range, bearing)  →  Para cada landmark:
    1. Data association: encontrar poste i en el mapa
    2. Innovación:  ν = z - h(μ̄, poste_i)
    3. Ganancia:    K = Σ̄·Hᵀ·(H·Σ̄·Hᵀ + R)⁻¹
    4. Corrección:  μ = μ̄ + K·ν
                    Σ = (I - K·H)·Σ̄
    publicar TF map→base_link_ekf con μ actualizado
```
