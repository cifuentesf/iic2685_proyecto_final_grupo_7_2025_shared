#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        
        # Parámetros de navegación
        self.linear_speed = 2.15
        self.angular_speed = 0.5
        self.desired_wall_distance = 1.5
        self.collision_distance = 0.75
        
        # Parámetros PID para seguimiento de pared
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.5
        self.integral_error = 0.0
        self.previous_error = 0.0
        
        # Estado del navegador
        self.current_scan = None
        
        # Regiones del LIDAR (5 sectores)
        self.regions = {
            'right': 0.0,
            'right_center': 0.0,
            'center': 0.0,
            'left_center': 0.0,
            'left': 0.0
        }
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        # Publisher para comandos de navegación
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer para control
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Navegador iniciado - Navegación siguiendo muralla derecha")

    def laser_callback(self, msg):
        self.current_scan = msg
        self.process_scan_regions(msg)

    def process_scan_regions(self, scan):
        """Procesa el scan láser y divide en 5 regiones"""
        if scan is None:
            return
        
        ranges = np.array(scan.ranges)
        # Limpiar datos inválidos
        ranges[~np.isfinite(ranges)] = 4.0
        ranges[ranges <= 0.0] = 4.0
        ranges[ranges > 4.0] = 4.0
        
        n = len(ranges)
        if n == 0:
            return
            
        sector_size = n // 5
        self.regions = {
            'right': np.min(ranges[0:sector_size]) if sector_size > 0 else 10.0,
            'right_center': np.min(ranges[sector_size:2*sector_size]) if sector_size > 0 else 10.0,
            'center': np.min(ranges[2*sector_size:3*sector_size]) if sector_size > 0 else 10.0,
            'left_center': np.min(ranges[3*sector_size:4*sector_size]) if sector_size > 0 else 10.0,
            'left': np.min(ranges[4*sector_size:]) if n > 4*sector_size else 10.0
        }

    def control_loop(self):
        """Loop principal de control"""
        if self.current_scan is None:
            return
            
        self.intelligent_navigation()

    def intelligent_navigation(self):
        """Navegación inteligente siguiendo muralla derecha"""
        cmd = Twist()
        
        if self.is_collision_imminent():
            self.get_logger().info("Evitando colisión")
            cmd = self.avoid_collision()
        elif self.has_right_wall():
            cmd = self.follow_right_wall()
        else:
            self.get_logger().info("Buscando pared derecha")
            cmd = self.search_right_wall()
            
        self.cmd_pub.publish(cmd)

    def is_collision_imminent(self):
        """Detecta si hay riesgo de colisión inminente"""
        return (self.regions['center'] < self.collision_distance or
                self.regions['left_center'] < self.collision_distance or
                self.regions['right_center'] < self.collision_distance)

    def avoid_collision(self):
        """Evita colisiones girando hacia el lado con más espacio"""
        cmd = Twist()
        cmd.linear.x = 0.0
        
        left_space = (self.regions['left'] + self.regions['left_center']) / 2
        right_space = (self.regions['right'] + self.regions['right_center']) / 2
        
        if left_space > right_space:
            cmd.angular.z = self.angular_speed
            self.get_logger().info("Rotando a la izquierda")
        else:
            cmd.angular.z = -self.angular_speed
            self.get_logger().info("Rotando a la derecha")
            
        return cmd

    def has_right_wall(self):
        """Verifica si hay una pared en el lado derecho"""
        return self.regions['right'] < 3.0

    def follow_right_wall(self):
        """Sigue la pared derecha usando control PID"""
        cmd = Twist()
        
        # Lógica especial si estamos entre dos paredes
        if (self.regions['left'] < self.collision_distance and 
            abs(self.regions['right'] - self.desired_wall_distance) < 0.1):
            target_distance = (self.regions['left'] + self.regions['right']) / 2
            error = self.regions['right'] - target_distance
            self.get_logger().info(f"Navegando entre paredes - Distancia objetivo: {target_distance:.2f}m")
        else:
            # Error para seguir pared derecha (negativo cuando está muy cerca, positivo cuando está lejos)
            error = self.regions['right'] - self.desired_wall_distance
        
        # Control PID
        self.integral_error += error * 0.05
        derivative_error = error - self.previous_error
        
        # Limitamos la integral para evitar windup
        self.integral_error = max(-1.0, min(1.0, self.integral_error))
        
        angular_velocity = -(self.kp * error + 
                           self.ki * self.integral_error + 
                           self.kd * derivative_error)
        
        # Limitamos la velocidad angular
        angular_velocity = max(-self.angular_speed, min(self.angular_speed, angular_velocity))
        
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_velocity
        
        self.previous_error = error
        
        # Log periódico para debugging
        if hasattr(self, '_last_log_time'):
            if time.time() - self._last_log_time > 1.0:
                self.get_logger().info(
                    f"Siguiendo pared der: {self.regions['right']:.2f}m "
                    f"(objetivo: {self.desired_wall_distance:.2f}m)"
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()
            
        return cmd

    def search_right_wall(self):
        """Busca una pared en el lado derecho"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed * 0.5  # Velocidad reducida durante búsqueda
        cmd.angular.z = -self.angular_speed * 0.3  # Giro lento hacia la derecha
        return cmd

    def stop_robot(self):
        """Detiene el robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop_robot()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()