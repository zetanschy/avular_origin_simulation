#!/usr/bin/env python3
"""
Go To Goal Controller for Avular Origin One

Controla el robot Avular para ir a una posición objetivo (x, y).
Soporta dos modos:
- skid_steer: control directo (v y ω simultáneos)
- mecanum: movimiento omnidireccional

Uso:
    ros2 run origin_one_gazebo go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0
    ros2 run origin_one_gazebo go_to_goal --ros-args -p goal_x:=3.0 -p goal_y:=2.0 -p mode:=mecanum
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def euler_from_quaternion(q):
    """Convierte quaternion a euler (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


def normalize_angle(angle):
    """Normaliza ángulo a [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        
        # Parámetros del controlador
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('mode', 'skid_steer')  # 'skid_steer' o 'mecanum'
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('distance_tolerance', 0.1)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.mode = self.get_parameter('mode').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        
        # Estado actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_reached = False
        
        # Pose inicial (para compensar offset de spawn en mecanum)
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        self.initialized = False
        
        # Publishers y Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/robot/odom', self.odom_callback, 10)
        
        # Timer de control (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Go To Goal iniciado - Modo: {self.mode}')
        self.get_logger().info(f'Objetivo: ({self.goal_x:.2f}, {self.goal_y:.2f})')
    
    def odom_callback(self, msg):
        """Actualiza la pose del robot desde odometría.
        
        Nota: El plugin OdometryPublisher (usado en mecanum) publica la pose
        absoluta en Gazebo, no relativa al inicio. Por eso restamos la pose inicial.
        """
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        _, _, raw_theta = euler_from_quaternion(msg.pose.pose.orientation)
        
        # Guardar pose inicial en el primer mensaje
        if not self.initialized:
            self.initial_x = raw_x
            self.initial_y = raw_y
            self.initial_theta = raw_theta
            self.initialized = True
            self.get_logger().info(
                f'Pose inicial: ({self.initial_x:.2f}, {self.initial_y:.2f}, {math.degrees(self.initial_theta):.1f}°)'
            )
        
        # Calcular pose relativa al inicio
        # Rotar el desplazamiento por -initial_theta para alinear con frame inicial
        dx = raw_x - self.initial_x
        dy = raw_y - self.initial_y
        cos_init = math.cos(-self.initial_theta)
        sin_init = math.sin(-self.initial_theta)
        
        self.x = dx * cos_init - dy * sin_init
        self.y = dx * sin_init + dy * cos_init
        self.theta = normalize_angle(raw_theta - self.initial_theta)
    
    def control_loop(self):
        """Loop principal de control."""
        if not self.initialized or self.goal_reached:
            return
        
        if self.mode == 'mecanum':
            self.control_mecanum()
        else:
            self.control_skid_steer()
    
    def control_skid_steer(self):
        """Control directo para Skid Steer.
        
        Aplica v y ω simultáneamente usando cos(angle_error) para modular v.
        """
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - self.theta)
        
        # Verificar si llegamos
        if distance < self.distance_tolerance:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info(f'Goal alcanzado! Posición: ({self.x:.2f}, {self.y:.2f})')
            return
        
        # Control proporcional para ω
        omega = self.kp_angular * angle_error
        omega = max(-self.angular_speed, min(self.angular_speed, omega))
        
        # Velocidad lineal: reducir si el error angular es grande
        # cos(angle_error): 1 si apunta al goal, 0 si perpendicular, -1 si opuesto
        v = self.kp_linear * distance * max(0, math.cos(angle_error))
        v = min(v, self.linear_speed)
        
        # Reducir velocidad cerca del objetivo
        if distance < 0.3:
            v = v * (distance / 0.3)
            v = max(v, 0.05)
        
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        
        self.cmd_pub.publish(cmd)
        self.log_status(distance, angle_error)
    
    def control_mecanum(self):
        """Control para modo Mecanum (omnidireccional).
        
        Se mueve directamente hacia el objetivo usando velocidad lateral.
        """
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Verificar si llegamos
        if distance < self.distance_tolerance:
            self.stop_robot()
            self.goal_reached = True
            self.get_logger().info(f'Goal alcanzado! Posición: ({self.x:.2f}, {self.y:.2f})')
            return
        
        # Ángulo hacia el goal en frame mundo
        angle_to_goal = math.atan2(dy, dx)
        
        # Transformar a frame del robot
        angle_in_robot_frame = normalize_angle(angle_to_goal - self.theta)
        
        # Calcular velocidades en frame del robot
        speed = self.kp_linear * distance
        speed = min(speed, self.linear_speed)
        
        vx = speed * math.cos(angle_in_robot_frame)
        vy = speed * math.sin(angle_in_robot_frame)
        
        # Opcional: rotar para mirar hacia el objetivo
        angle_error = normalize_angle(angle_to_goal - self.theta)
        omega = self.kp_angular * angle_error * 0.3  # rotación suave
        omega = max(-self.angular_speed, min(self.angular_speed, omega))
        
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega
        
        self.cmd_pub.publish(cmd)
        self.log_status(distance, angle_error)
    
    def log_status(self, distance, angle_error):
        """Log del estado actual."""
        self.get_logger().info(
            f'Pos: ({self.x:.2f}, {self.y:.2f}) | '
            f'Dist: {distance:.2f}m | '
            f'θ_err: {math.degrees(angle_error):.1f}°',
            throttle_duration_sec=0.5
        )
    
    def stop_robot(self):
        """Detiene el robot."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrumpido por usuario')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
