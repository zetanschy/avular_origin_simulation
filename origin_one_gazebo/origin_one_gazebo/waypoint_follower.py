#!/usr/bin/env python3
"""
Waypoint Follower para Avular Origin One (Skid Steer)

Sigue una secuencia de waypoints usando control directo.

Uso:
    ros2 run origin_one_gazebo waypoint_follower
    ros2 run origin_one_gazebo waypoint_follower --ros-args -p shape:=triangle -p size:=2.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def euler_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Parámetros
        self.declare_parameter('shape', 'square')
        self.declare_parameter('size', 3.0)
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.1)
        
        shape = self.get_parameter('shape').value
        self.size = self.get_parameter('size').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        # Generar waypoints
        self.waypoints = self.generate_waypoints(shape, self.size)
        self.current_idx = 0
        
        # Estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.finished = False
        
        # Pose inicial (para compensar offset de odometría)
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        self.initialized = False
        
        # Ganancias del controlador
        self.kp_linear = 0.8
        self.kp_angular = 1.5
        
        # ROS
        self.cmd_pub = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/robot/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f'Waypoint Follower (Skid Steer)')
        self.get_logger().info(f'Forma: {shape}, Tamaño: {self.size}m')
        self.get_logger().info(f'Waypoints: {self.waypoints}')
    
    def generate_waypoints(self, shape, size):
        """Genera waypoints según la forma."""
        if shape == 'square':
            return [
                (size, 0.0),
                (size, size),
                (0.0, size),
                (0.0, 0.0),
            ]
        elif shape == 'triangle':
            return [
                (size, 0.0),
                (size/2, size * 0.866),  # altura triángulo equilátero
                (0.0, 0.0),
            ]
        elif shape == 'line':
            return [
                (size, 0.0),
                (0.0, 0.0),
            ]
        else:
            return [
                (size, 0.0),
                (size, size),
                (0.0, size),
                (0.0, 0.0),
            ]
    
    def odom_callback(self, msg):
        """Callback de odometría - calcula pose relativa al inicio."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_theta = euler_from_quaternion(msg.pose.pose.orientation)
        
        # Guardar pose inicial
        if not self.initialized:
            self.initial_x = raw_x
            self.initial_y = raw_y
            self.initial_theta = raw_theta
            self.initialized = True
            self.get_logger().info(f'Pose inicial: ({raw_x:.2f}, {raw_y:.2f})')
        
        # Calcular pose relativa al inicio
        dx = raw_x - self.initial_x
        dy = raw_y - self.initial_y
        cos_init = math.cos(-self.initial_theta)
        sin_init = math.sin(-self.initial_theta)
        
        self.x = dx * cos_init - dy * sin_init
        self.y = dx * sin_init + dy * cos_init
        self.theta = normalize_angle(raw_theta - self.initial_theta)
    
    def control_loop(self):
        """Loop de control - ejecuta a 20Hz."""
        if not self.initialized or self.finished:
            return
        
        # Verificar si completamos todos los waypoints
        if self.current_idx >= len(self.waypoints):
            self.stop_robot()
            self.finished = True
            self.get_logger().info('¡Ruta completada!')
            return
        
        # Obtener waypoint actual
        goal_x, goal_y = self.waypoints[self.current_idx]
        
        # Calcular error de posición
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # ¿Llegamos al waypoint?
        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Waypoint {self.current_idx + 1}/{len(self.waypoints)} alcanzado!')
            self.current_idx += 1
            return
        
        # Calcular error angular
        angle_to_goal = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - self.theta)
        
        # ============================================
        # CONTROL SKID STEER (v y ω simultáneos)
        # ============================================
        
        # Control proporcional para velocidad angular
        omega = self.kp_angular * angle_error
        omega = max(-self.angular_speed, min(self.angular_speed, omega))
        
        # Velocidad lineal modulada por cos(angle_error)
        # Si apunta al goal: cos(0)=1 -> velocidad máxima
        # Si perpendicular: cos(90)=0 -> solo gira
        v = self.kp_linear * distance * max(0, math.cos(angle_error))
        v = min(v, self.linear_speed)
        
        # Publicar comando
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)
        
        # Log
        self.get_logger().info(
            f'WP {self.current_idx + 1}/{len(self.waypoints)} | '
            f'Pos: ({self.x:.2f}, {self.y:.2f}) | '
            f'Dist: {distance:.2f}m',
            throttle_duration_sec=0.5
        )
    
    def stop_robot(self):
        """Detiene el robot."""
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
