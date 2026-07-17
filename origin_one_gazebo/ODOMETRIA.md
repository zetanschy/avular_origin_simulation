# Odometría del Robot Avular Origin One

Este documento explica en detalle cómo funciona la odometría para el robot Avular Origin One, que soporta dos configuraciones de tracción: **Skid Steer** (diferencial) y **Mecanum** (omnidireccional).

---

## Índice

1. [Introducción a la Odometría](#1-introducción-a-la-odometría)
2. [Parámetros del Robot](#2-parámetros-del-robot)
3. [Odometría Skid Steer (Diferencial)](#3-odometría-skid-steer-diferencial)
4. [Odometría Mecanum (Omnidireccional)](#4-odometría-mecanum-omnidireccional)
5. [Implementación en Gazebo](#5-implementación-en-gazebo)
6. [Topics de ROS](#6-topics-de-ros)
7. [Visualización en RViz](#7-visualización-en-rviz)

---

## 1. Introducción a la Odometría

La **odometría** es el método de estimación de la posición de un robot móvil basándose en el movimiento de sus ruedas. Es una técnica fundamental en robótica móvil que permite:

- Estimar la posición (x, y) del robot
- Estimar la orientación (θ) del robot
- Calcular velocidades lineales y angulares

### Ventajas
- Bajo costo computacional
- Alta frecuencia de actualización
- No requiere sensores externos

### Desventajas
- Error acumulativo (drift)
- Sensible al deslizamiento de ruedas
- Requiere calibración precisa

---

## 2. Parámetros del Robot

El robot Avular Origin One tiene diferentes parámetros según la configuración de tracción:

### Skid Steer Drive
| Parámetro | Símbolo | Valor |
|-----------|---------|-------|
| Ancho de vía (Track Width) | L | 0.500 m |
| Distancia entre ejes (Wheelbase) | W | 0.410 m |
| Radio de rueda | r | 0.120 m |
| Separación efectiva* | L_eff | 0.450 m |

*La separación efectiva es el valor usado en el cálculo de odometría, ajustado para compensar el deslizamiento.

### Mecanum Drive
| Parámetro | Símbolo | Valor |
|-----------|---------|-------|
| Ancho de vía (Track Width) | L | 0.475 m |
| Distancia entre ejes (Wheelbase) | W | 0.410 m |
| Radio de rueda | r | 0.1015 m |

---

## 3. Odometría Skid Steer (Diferencial)

### 3.1 Cinemática del Robot

En un robot de tracción diferencial/skid-steer, el movimiento se controla variando las velocidades de las ruedas izquierdas y derechas.

```
        FRENTE
    ┌─────────────┐
    │  FL     FR  │    FL = Front Left
    │      ↑      │    FR = Front Right
    │      │      │    RL = Rear Left
    │   base_link │    RR = Rear Right
    │             │
    │  RL     RR  │
    └─────────────┘
         ← L →
```

### 3.2 Cinemática Directa (Forward Kinematics)

De las velocidades de las ruedas a las velocidades del robot:

```
Velocidad lineal:      v = (v_R + v_L) / 2
Velocidad angular:     ω = (v_R - v_L) / L
```

Donde:
- `v_L` = velocidad lineal de las ruedas izquierdas = ω_L × r
- `v_R` = velocidad lineal de las ruedas derechas = ω_R × r
- `L` = separación entre ruedas (track width)
- `r` = radio de la rueda

### 3.3 Cinemática Inversa (Inverse Kinematics)

De los comandos cmd_vel a las velocidades de las ruedas:

```
v_L = v - (ω × L) / 2
v_R = v + (ω × L) / 2
```

O en términos de velocidad angular de ruedas:

```
ω_L = (v - (ω × L) / 2) / r
ω_R = (v + (ω × L) / 2) / r
```

### 3.4 Actualización de Pose (Integración)

Para actualizar la posición del robot, integramos las velocidades:

```python
# Método de Euler (simple)
Δt = tiempo_actual - tiempo_anterior

# Si ω ≈ 0 (movimiento recto)
if abs(ω) < 0.001:
    Δx = v × cos(θ) × Δt
    Δy = v × sin(θ) × Δt
    Δθ = 0

# Si ω ≠ 0 (movimiento curvo)
else:
    Δθ = ω × Δt
    Δx = v/ω × (sin(θ + Δθ) - sin(θ))
    Δy = v/ω × (cos(θ) - cos(θ + Δθ))

# Actualizar pose
x = x + Δx
y = y + Δy
θ = θ + Δθ
```

### 3.5 Ejemplo de Código Python

```python
import math

class SkidSteerOdometry:
    def __init__(self, wheel_radius=0.120, track_width=0.450):
        self.r = wheel_radius
        self.L = track_width
        
        # Pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_time = None
        
    def update_from_wheel_velocities(self, omega_left, omega_right, current_time):
        """
        Actualiza la odometría a partir de velocidades angulares de ruedas.
        
        Args:
            omega_left: velocidad angular ruedas izquierdas (rad/s)
            omega_right: velocidad angular ruedas derechas (rad/s)
            current_time: tiempo actual (segundos)
        """
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Velocidades lineales de las ruedas
        v_left = omega_left * self.r
        v_right = omega_right * self.r
        
        # Velocidades del robot
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.L
        
        # Integración de la pose
        if abs(omega) < 1e-6:
            # Movimiento recto
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            # Movimiento curvo
            delta_theta = omega * dt
            self.x += v/omega * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += v/omega * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
            self.theta += delta_theta
        
        # Normalizar theta a [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return self.x, self.y, self.theta, v, omega
    
    def update_from_cmd_vel(self, v_cmd, omega_cmd, current_time):
        """
        Actualiza la odometría directamente desde cmd_vel.
        
        Args:
            v_cmd: velocidad lineal comandada (m/s)
            omega_cmd: velocidad angular comandada (rad/s)
            current_time: tiempo actual (segundos)
        """
        # Convertir a velocidades de ruedas
        v_left = v_cmd - (omega_cmd * self.L / 2.0)
        v_right = v_cmd + (omega_cmd * self.L / 2.0)
        
        omega_left = v_left / self.r
        omega_right = v_right / self.r
        
        return self.update_from_wheel_velocities(omega_left, omega_right, current_time)
```

---

## 4. Odometría Mecanum (Omnidireccional)

### 4.1 Cinemática del Robot Mecanum

Los robots con ruedas Mecanum pueden moverse en cualquier dirección sin girar. Cada rueda tiene rodillos a 45° que permiten movimiento omnidireccional.

```
        FRENTE
    ┌─────────────┐
    │  ⟋FL   FR⟍  │    FL: rodillos a 45° (tipo A)
    │      ↑      │    FR: rodillos a -45° (tipo B)
    │      │→ Y   │    RL: rodillos a -45° (tipo B)
    │   base_link │    RR: rodillos a 45° (tipo A)
    │      X      │
    │  ⟍RL   RR⟋  │    (patrón X estándar)
    └─────────────┘
         ← L →
         
    ↑
    W (wheelbase)
    ↓
```

### 4.2 Direcciones de Fricción en las Ruedas

El robot Avular usa el patrón de fricción definido en el URDF:

| Rueda | Dirección de fricción (fdir1) |
|-------|------------------------------|
| FL (Front Left) | (1, -1, 0) normalizado |
| FR (Front Right) | (1, 1, 0) normalizado |
| RL (Rear Left) | (1, 1, 0) normalizado |
| RR (Rear Right) | (1, -1, 0) normalizado |

### 4.3 Cinemática Directa

De las velocidades de las ruedas a las velocidades del robot:

```
v_x = (r/4) × (ω_FL + ω_FR + ω_RL + ω_RR)
v_y = (r/4) × (-ω_FL + ω_FR + ω_RL - ω_RR)
ω   = (r/4) × (-ω_FL + ω_FR - ω_RL + ω_RR) / (L/2 + W/2)
```

Donde:
- `v_x` = velocidad en X (adelante/atrás)
- `v_y` = velocidad en Y (izquierda/derecha)
- `ω` = velocidad angular (rotación)
- `L` = ancho de vía (track width)
- `W` = distancia entre ejes (wheelbase)

### 4.4 Cinemática Inversa

De cmd_vel a velocidades de ruedas:

```
k = L/2 + W/2  (factor geométrico)

ω_FL = (1/r) × (v_x - v_y - k×ω)
ω_FR = (1/r) × (v_x + v_y + k×ω)
ω_RL = (1/r) × (v_x + v_y - k×ω)
ω_RR = (1/r) × (v_x - v_y + k×ω)
```

### 4.5 Actualización de Pose

Para el robot omnidireccional, la actualización de pose considera movimiento en X, Y y rotación:

```python
# Transformar velocidades del frame del robot al frame del mundo
Δt = tiempo_actual - tiempo_anterior

# Velocidades en el frame del mundo
v_x_world = v_x × cos(θ) - v_y × sin(θ)
v_y_world = v_x × sin(θ) + v_y × cos(θ)

# Actualizar pose
x = x + v_x_world × Δt
y = y + v_y_world × Δt
θ = θ + ω × Δt
```

### 4.6 Ejemplo de Código Python

```python
import math

class MecanumOdometry:
    def __init__(self, wheel_radius=0.1015, track_width=0.475, wheelbase=0.410):
        self.r = wheel_radius
        self.L = track_width      # Ancho entre ruedas
        self.W = wheelbase        # Distancia entre ejes
        self.k = (self.L + self.W) / 2.0  # Factor geométrico
        
        # Pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Velocidades
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        
        self.last_time = None
        
    def update_from_wheel_velocities(self, omega_fl, omega_fr, omega_rl, omega_rr, current_time):
        """
        Actualiza la odometría a partir de velocidades angulares de las 4 ruedas.
        
        Args:
            omega_fl: velocidad angular rueda frontal izquierda (rad/s)
            omega_fr: velocidad angular rueda frontal derecha (rad/s)
            omega_rl: velocidad angular rueda trasera izquierda (rad/s)
            omega_rr: velocidad angular rueda trasera derecha (rad/s)
            current_time: tiempo actual (segundos)
        """
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Cinemática directa: de velocidades de ruedas a velocidades del robot
        # Estas fórmulas dependen de la configuración de los rodillos
        self.vx = (self.r / 4.0) * (omega_fl + omega_fr + omega_rl + omega_rr)
        self.vy = (self.r / 4.0) * (-omega_fl + omega_fr + omega_rl - omega_rr)
        self.omega = (self.r / (4.0 * self.k)) * (-omega_fl + omega_fr - omega_rl + omega_rr)
        
        # Transformar velocidades del frame del robot al frame del mundo
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        
        vx_world = self.vx * cos_theta - self.vy * sin_theta
        vy_world = self.vx * sin_theta + self.vy * cos_theta
        
        # Integrar para obtener la pose
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += self.omega * dt
        
        # Normalizar theta a [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        return self.x, self.y, self.theta, self.vx, self.vy, self.omega
    
    def update_from_cmd_vel(self, vx_cmd, vy_cmd, omega_cmd, current_time):
        """
        Actualiza la odometría directamente desde cmd_vel.
        
        Args:
            vx_cmd: velocidad lineal X comandada (m/s)
            vy_cmd: velocidad lineal Y comandada (m/s)
            omega_cmd: velocidad angular comandada (rad/s)
            current_time: tiempo actual (segundos)
        """
        # Cinemática inversa: de cmd_vel a velocidades de ruedas
        omega_fl = (1.0 / self.r) * (vx_cmd - vy_cmd - self.k * omega_cmd)
        omega_fr = (1.0 / self.r) * (vx_cmd + vy_cmd + self.k * omega_cmd)
        omega_rl = (1.0 / self.r) * (vx_cmd + vy_cmd - self.k * omega_cmd)
        omega_rr = (1.0 / self.r) * (vx_cmd - vy_cmd + self.k * omega_cmd)
        
        return self.update_from_wheel_velocities(omega_fl, omega_fr, omega_rl, omega_rr, current_time)
    
    def get_wheel_velocities_from_cmd(self, vx_cmd, vy_cmd, omega_cmd):
        """
        Calcula las velocidades de ruedas necesarias para un cmd_vel dado.
        Útil para debug o visualización.
        """
        omega_fl = (1.0 / self.r) * (vx_cmd - vy_cmd - self.k * omega_cmd)
        omega_fr = (1.0 / self.r) * (vx_cmd + vy_cmd + self.k * omega_cmd)
        omega_rl = (1.0 / self.r) * (vx_cmd + vy_cmd - self.k * omega_cmd)
        omega_rr = (1.0 / self.r) * (vx_cmd - vy_cmd + self.k * omega_cmd)
        
        return {
            'front_left': omega_fl,
            'front_right': omega_fr,
            'rear_left': omega_rl,
            'rear_right': omega_rr
        }
```

---

## 5. Implementación en Gazebo

### 5.1 Skid Steer Drive

El robot usa el plugin `gz-sim-diff-drive-system`:

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>main_body_left_front_wheel</left_joint>
    <left_joint>main_body_left_rear_wheel</left_joint>
    <right_joint>main_body_right_front_wheel</right_joint>
    <right_joint>main_body_right_rear_wheel</right_joint>
    <wheel_separation>0.45</wheel_separation>
    <wheel_radius>0.120</wheel_radius>
    <max_linear_velocity>1.5</max_linear_velocity>
    <max_angular_velocity>2</max_angular_velocity>
    <topic>/robot/cmd_vel</topic>
    <odom_topic>/robot/odom</odom_topic>
    <tf_topic>/robot/tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
</plugin>
```

Este plugin:
- Recibe comandos en `/robot/cmd_vel` (Twist)
- Publica odometría en `/robot/odom` (Odometry)
- Publica transformación odom→base_link en `/robot/tf`

### 5.2 Mecanum Drive

El robot mecanum usa dos plugins separados:

**1. Plugin de control:**
```xml
<plugin filename="gz-sim-mecanum-drive-system" name="ignition::gazebo::systems::MecanumDrive">
    <front_left_joint>main_body_left_front_wheel</front_left_joint>
    <front_right_joint>main_body_right_front_wheel</front_right_joint>
    <back_left_joint>main_body_left_rear_wheel</back_left_joint>
    <back_right_joint>main_body_right_rear_wheel</back_right_joint>
    <wheel_separation>0.475</wheel_separation>
    <wheelbase>0.410</wheelbase>
    <wheel_radius>0.1015</wheel_radius>
    <topic>/robot/cmd_vel</topic>
</plugin>
```

**2. Plugin de odometría (separado):**
```xml
<plugin filename="gz-sim-odometry-publisher-system" name="ignition::gazebo::systems::OdometryPublisher">
    <odom_topic>/robot/odom</odom_topic>
    <tf_topic>/robot/tf</tf_topic>
    <odom_frame>odom</odom_frame>
    <robot_base_frame>base_link</robot_base_frame>
</plugin>
```

> **Nota:** El plugin mecanum no tiene odometría integrada, por eso se usa un plugin separado.

### 5.3 Diferencia Importante: Origen de la Odometría

Existe una diferencia fundamental entre cómo calculan la odometría ambos plugins:

| Característica | DiffDrive | OdometryPublisher |
|----------------|-----------|-------------------|
| **Método de cálculo** | Integración de velocidades de ruedas | Lectura directa de pose en Gazebo |
| **Origen del frame odom** | Siempre (0, 0, 0) | Posición de spawn del robot |
| **Tipo de dato** | Dead reckoning (estimación) | Ground truth (verdad absoluta) |

**¿Por qué ocurre esto?**

1. **DiffDrive Plugin**: Calcula la odometría integrando las velocidades de las ruedas en el tiempo. El frame `odom` se inicializa en (0, 0, 0) cuando el plugin arranca, independientemente de dónde esté el robot en el mundo de Gazebo.

   ```
   Spawn en (5, 3, 0) en Gazebo → Odometría inicia en (0, 0, 0)
   Robot se mueve 1m adelante → Odometría: (1, 0, 0)
   ```

2. **OdometryPublisher Plugin**: Lee la pose real del modelo directamente desde el estado interno de Gazebo. Si el robot fue spawneado en una posición específica, esa posición se refleja en la odometría.

   ```
   Spawn en (5, 3, 0) en Gazebo → Odometría inicia en (5, 3, 0)
   Robot se mueve 1m adelante → Odometría: (6, 3, 0)
   ```
---

## 6. Topics de ROS

### 6.1 Topics de Entrada

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/robot/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidad |

**Estructura de Twist:**
```yaml
linear:
  x: float64  # Velocidad adelante/atrás (m/s)
  y: float64  # Velocidad lateral (m/s) - solo mecanum
  z: float64  # No usado
angular:
  x: float64  # No usado
  y: float64  # No usado
  z: float64  # Velocidad de rotación (rad/s)
```

### 6.2 Topics de Salida

| Topic | Tipo | Descripción |
|-------|------|-------------|
| `/robot/odom` | `nav_msgs/Odometry` | Odometría del robot |
| `/robot/tf` | `tf2_msgs/TFMessage` | Transformación odom→base_link |
| `/robot/joint_states` | `sensor_msgs/JointState` | Estados de los joints |

**Estructura de Odometry:**
```yaml
header:
  stamp: time
  frame_id: "odom"
child_frame_id: "base_link"
pose:
  pose:
    position: {x, y, z}
    orientation: {x, y, z, w}  # Quaternion
  covariance: float64[36]
twist:
  twist:
    linear: {x, y, z}
    angular: {x, y, z}
  covariance: float64[36]
```

### 6.3 Ejemplo: Leer Odometría

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/odom',
            self.odom_callback,
            10)
    
    def odom_callback(self, msg):
        # Posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Orientación (quaternion a euler)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # Velocidades
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        self.get_logger().info(
            f'Pose: ({x:.3f}, {y:.3f}, {yaw:.3f}) | '
            f'Vel: ({vx:.3f}, {vy:.3f}, {omega:.3f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

---

## 7. Visualización en RViz

### 7.1 Lanzar la Simulación y RViz

**Paso 1: Lanzar la simulación en Gazebo**

```bash
# Terminal 1: Lanzar Gazebo con el robot (Skid Steer)
ros2 launch origin_one_gazebo ty_test_area.launch.py drive_configuration:=skid_steer_drive

# O para Mecanum:
ros2 launch origin_one_gazebo ty_test_area.launch.py drive_configuration:=mecanum_drive
```

**Paso 2: Lanzar RViz en otra terminal**

```bash
# Terminal 2: Lanzar RViz con configuración del robot
ros2 launch origin_one_description origin_one_rviz.launch.py
```

## Referencias

- [Gazebo DiffDrive Plugin](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
- [Gazebo MecanumDrive Plugin](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MecanumDrive.html)
- [Gazebo OdometryPublisher Plugin](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)
- [ROS nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)
- Siegwart, R., & Nourbakhsh, I. R. (2004). Introduction to Autonomous Mobile Robots

---