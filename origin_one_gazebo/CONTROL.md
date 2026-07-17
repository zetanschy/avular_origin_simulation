# Control del Robot Avular Origin One

## Parámetros del Robot

### Skid Steer
| Parámetro | Valor |
|-----------|-------|
| Track Width | 0.500 m |
| Wheelbase | 0.410 m |
| Wheel Radius | 0.120 m |
| Wheel Separation (efectiva) | 0.450 m |

### Mecanum
| Parámetro | Valor |
|-----------|-------|
| Track Width | 0.475 m |
| Wheelbase | 0.410 m |
| Wheel Radius | 0.1015 m |

## Topics

| Topic | Tipo | Dirección |
|-------|------|-----------|
| `/robot/cmd_vel` | `geometry_msgs/Twist` | Entrada |
| `/robot/odom` | `nav_msgs/Odometry` | Salida |
| `/robot/lidar/points` | `sensor_msgs/PointCloud2` | Salida |

## Diferencias entre Modos

| Característica | Skid Steer | Mecanum |
|----------------|------------|---------|
| Giro en el lugar | Sí | Sí |
| Movimiento lateral | No | Sí |
| `linear.y` en cmd_vel | Ignorado | Usado |
| Deslizamiento | Alto | Bajo |

## Lanzar Simulación

```bash
# Terminal 1: Gazebo (Skid Steer)
ros2 launch origin_one_gazebo ty_test_area.launch.py drive_configuration:=skid_steer_drive

# Terminal 1: Gazebo (Mecanum)
ros2 launch origin_one_gazebo ty_test_area.launch.py drive_configuration:=mecanum_drive

# Terminal 2: RViz
ros2 launch origin_one_description origin_one_rviz.launch.py
```

## Ejecutar Go To Goal

```bash
# Skid Steer (default)
ros2 run origin_one_gazebo go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0

# Mecanum
ros2 run origin_one_gazebo go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0 -p mode:=mecanum
```

### Parámetros del Controlador

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `goal_x` | 2.0 | Posición X objetivo (metros) |
| `goal_y` | 0.0 | Posición Y objetivo (metros) |
| `mode` | skid_steer | Modo: `skid_steer` o `mecanum` |
| `linear_speed` | 0.5 | Velocidad lineal máxima (m/s) |
| `angular_speed` | 0.8 | Velocidad angular máxima (rad/s) |
| `kp_linear` | 0.8 | Ganancia proporcional lineal |
| `kp_angular` | 1.5 | Ganancia proporcional angular |
| `distance_tolerance` | 0.1 | Tolerancia de distancia (m) |

---

## Explicación del Script `go_to_goal.py`

### Resumen de Modos

| Modo | Estrategia | Ventaja |
|------|------------|---------|
| `skid_steer` | v y ω simultáneos | Trayectorias suaves, eficiente |
| `mecanum` | vx, vy, ω independientes | Más rápido, movimiento lateral |

---

### Modo Skid Steer

Estrategia: **Control Directo** (v y ω simultáneos)

```
                    Goal
                      ★
                     /
                    / angle_error
                   /
            ┌─────┴─────┐
            │   Robot   │
            │     θ     │
            └───────────┘

    ω = Kp_angular * angle_error
    v = Kp_linear * distance * cos(angle_error)
```

**Funcionamiento:**
- `ω` siempre corrige hacia el objetivo
- `v` se modula con `cos(angle_error)`:

| angle_error | cos() | Velocidad | Comportamiento |
|-------------|-------|-----------|----------------|
| 0° | 1.0 | 100% | Avanza directo |
| 45° | 0.7 | 70% | Avanza + gira |
| 90° | 0.0 | 0% | Solo gira |
| 180° | -1.0 | 0% | Solo gira |

---

### Modo Mecanum

Estrategia: **Movimiento Omnidireccional**

```
                    Goal (x_g, y_g)
                         ★
                        /
                       / d = distancia
                      /
                     /  α = ángulo en frame robot
                    /
              ┌────┴────┐
              │  Robot  │ → vx = speed * cos(α)
              │   θ     │ → vy = speed * sin(α)
              └─────────┘
```

**Ventaja:** Llega más rápido usando movimiento lateral.

**Nota sobre odometría:** El plugin `OdometryPublisher` usado en mecanum publica 
la pose absoluta en Gazebo (incluyendo posición de spawn). El script compensa esto 
automáticamente guardando la pose inicial y calculando coordenadas relativas.

---

### Diagrama del Algoritmo

```
              ┌───────────────────────┐
              │  Leer odometría       │
              │  (x, y, θ)            │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  Calcular:            │
              │  - distancia al goal  │
              │  - ángulo al goal     │
              │  - error angular      │
              └───────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │  ¿distancia < tol?    │──── Sí ───▶ STOP
              └───────────────────────┘
                          │ No
                          ▼
              ┌───────────────────────┐
              │     Según modo:       │
              └───────────────────────┘
                    │           │
        skid_steer  │           │  mecanum
                    │           │
                    ▼           ▼
           ┌────────────┐  ┌────────────┐
           │  v*cos(e)  │  │  vx, vy    │
           │  + ω       │  │  + ω       │
           └────────────┘  └────────────┘
                    │           │
                    └─────┬─────┘
                          ▼
              ┌───────────────────────┐
              │  Publicar cmd_vel     │
              └───────────────────────┘
```

---

### Código Principal

**1. Cálculo del error:**
```python
dx = goal_x - x
dy = goal_y - y
distance = sqrt(dx² + dy²)
angle_to_goal = atan2(dy, dx)
angle_error = angle_to_goal - theta
```

**2. Control Skid Steer:**
```python
omega = Kp_angular * angle_error
v = Kp_linear * distance * cos(angle_error)
```

**3. Control Mecanum:**
```python
angle_in_robot = angle_to_goal - theta
vx = speed * cos(angle_in_robot)
vy = speed * sin(angle_in_robot)
```

---

## Comandos Útiles

```bash
# Ver odometría
ros2 topic echo /robot/odom

# Ver TF
ros2 run tf2_ros tf2_echo odom base_link

# Movimiento manual - adelante
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"

# Movimiento manual - rotación
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Movimiento manual - lateral (solo mecanum)
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.5}}"

# Detener
ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## Comparación: Ackermann vs Skid Steer vs Mecanum

| Característica | Ackermann | Skid Steer | Mecanum |
|----------------|-----------|------------|---------|
| Giro en lugar | No | Sí | Sí |
| Mov. lateral | No | No | Sí |
| Deslizamiento | Bajo | Alto | Bajo |
| Control | Complejo | Simple | Simple |
| Radio mín. giro | L/tan(δ) | 0 | 0 |
