# Seguimiento de Trayectorias - Avular Origin One

## Waypoint Follower (Skid Steer)

Script de referencia que sigue una secuencia de waypoints usando control directo.

### Uso

```bash
# Lanzar simulación
ros2 launch origin_one_gazebo ty_test_area.launch.py drive_configuration:=skid_steer_drive

# Ejecutar waypoint follower (cuadrado por defecto)
ros2 run origin_one_gazebo waypoint_follower

# Triángulo
ros2 run origin_one_gazebo waypoint_follower --ros-args -p shape:=triangle

# Cambiar tamaño
ros2 run origin_one_gazebo waypoint_follower --ros-args -p shape:=square -p size:=2.0
```

### Parámetros

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `shape` | square | Forma: `square`, `triangle`, `line` |
| `size` | 3.0 | Tamaño de la figura (metros) |
| `linear_speed` | 0.4 | Velocidad lineal máxima (m/s) |
| `angular_speed` | 0.8 | Velocidad angular máxima (rad/s) |
| `waypoint_tolerance` | 0.1 | Tolerancia para alcanzar waypoint (m) |

---

## Algoritmo de Control (Skid Steer)

```
┌─────────────────────────────────────────────────────────┐
│                   CONTROL DIRECTO                        │
└─────────────────────────────────────────────────────────┘

1. Calcular error de posición:
   dx = goal_x - robot_x
   dy = goal_y - robot_y
   distance = sqrt(dx² + dy²)

2. Calcular error angular:
   angle_to_goal = atan2(dy, dx)
   angle_error = angle_to_goal - robot_theta

3. Control:
   ω = Kp_angular * angle_error
   v = Kp_linear * distance * cos(angle_error)
```

### Modulación de velocidad con cos(angle_error)

| angle_error | cos() | v | Comportamiento |
|-------------|-------|---|----------------|
| 0° | 1.0 | máxima | Avanza directo |
| 45° | 0.7 | 70% | Avanza + gira |
| 90° | 0.0 | 0 | Solo gira |
| 180° | -1.0 | 0 | Solo gira |

---

## Formas Disponibles

### Square (Cuadrado)
```
    (0,s)────────(s,s)
      │            │
      │            │
      │            │
    (0,0)────────(s,0)
     START
```

### Triangle (Triángulo equilátero)
```
         (s/2, s*0.866)
              ★
             / \
            /   \
           /     \
    (0,0)─────────(s,0)
    START
```

### Line (Línea)
```
    (0,0)────────────────(s,0)
    START                 END
```
