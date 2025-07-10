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
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.desired_wall_distance = 0.4
        self.collision_distance = 0.25
        
        # Parámetros PID para seguimiento de pared
        self.kp = 2.0
        self.ki = 0.1
        self.kd = 0.5
        self.integral_error = 0.0
        self.previous_error = 0.0
        
        # Estado del navegador
        self.current_scan = None
        self.autonomous_navigation_enabled = True
        
        # Variables para detección de teleoperación
        self.teleop_active = False
        self.last_cmd_vel_time = 0.0
        self.last_external_cmd = None
        self.cmd_vel_timeout = 1.5  # Si no hay comandos externos por 1.5s, reanudar
        
        # Regiones del LIDAR (5 sectores)
        self.regions = {
            'right': 0.0,
            'right_center': 0.0,
            'center': 0.0,
            'left_center': 0.0,
            'left': 0.0
        }
        
        # Variables para tracking de comandos propios
        self.my_last_cmd = Twist()
        self.cmd_tolerance = 0.001  # Tolerancia para comparar comandos flotantes
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        # Subscriber para monitorear cmd_vel (detectar teleop)
        self.cmd_vel_monitor = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_monitor_callback, 10)
        
        # Publisher para comandos del robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer para control y monitoreo
        self.create_timer(0.05, self.control_loop)
        self.create_timer(0.5, self.monitor_teleop_status)  # Monitorea cada 0.5 segundos
        
        self.get_logger().info("Navegador iniciado - Navegación siguiendo muralla izquierda")
        self.get_logger().info("Para pausar navegación, ejecuta: ros2 run teleop_twist_keyboard teleop_twist_keyboard")

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

    def commands_are_similar(self, cmd1, cmd2):
        """Compara si dos comandos Twist son similares (dentro de tolerancia)"""
        return (abs(cmd1.linear.x - cmd2.linear.x) < self.cmd_tolerance and
                abs(cmd1.angular.z - cmd2.angular.z) < self.cmd_tolerance)

    def cmd_vel_monitor_callback(self, msg):
        """Monitorea comandos en /cmd_vel para detectar teleoperación"""
        current_time = time.time()
        
        # Si el comando no es similar al último que enviamos, probablemente es teleop
        if not self.commands_are_similar(msg, self.my_last_cmd):
            # Comando externo detectado
            if msg.linear.x != 0.0 or msg.angular.z != 0.0:  # Solo si no es comando de parada
                self.last_external_cmd = msg
                self.last_cmd_vel_time = current_time
                
                if not self.teleop_active:
                    self.teleop_active = True
                    self.autonomous_navigation_enabled = False
                    self.get_logger().info("Teleoperación detectada - Navegación autónoma pausada")

    def monitor_teleop_status(self):
        """Monitorea si la teleoperación sigue activa"""
        current_time = time.time()
        
        # Si no hay comandos externos por un tiempo, reanudar navegación autónoma
        if self.teleop_active and (current_time - self.last_cmd_vel_time) > self.cmd_vel_timeout:
            self.teleop_active = False
            self.autonomous_navigation_enabled = True
            self.get_logger().info("Teleoperación terminada - Navegación autónoma reanudada")

    def control_loop(self):
        """Loop principal de control"""
        if self.current_scan is None:
            return
            
        # Solo navegar autónomamente si no hay teleoperación activa
        if self.autonomous_navigation_enabled:
            self.intelligent_navigation()

    def intelligent_navigation(self):
        """Navegación inteligente siguiendo muralla izquierda"""
        cmd = Twist()
        
        if self.is_collision_imminent():
            self.get_logger().info("Evitando colisión")
            cmd = self.avoid_collision()
        elif self.has_left_wall():
            cmd = self.follow_left_wall()
        else:
            self.get_logger().info("Buscando pared izquierda")
            cmd = self.search_left_wall()
        
        # Guardar el comando que vamos a enviar para poder identificarlo después
        self.my_last_cmd = cmd
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

    def has_left_wall(self):
        """Verifica si hay una pared en el lado izquierdo"""
        return self.regions['left'] < 3.0

    def follow_left_wall(self):
        """Sigue la pared izquierda usando control PID"""
        cmd = Twist()
        
        # Lógica especial si estamos entre dos paredes
        if (self.regions['right'] < self.collision_distance and 
            abs(self.regions['left'] - self.desired_wall_distance) < 0.1):
            target_distance = (self.regions['left'] + self.regions['right']) / 2
            error = target_distance - self.regions['left']
            self.get_logger().info(f"Navegando entre paredes - Distancia objetivo: {target_distance:.2f}m")
        else:
            error = self.desired_wall_distance - self.regions['left']
        
        # Control PID
        self.integral_error += error * 0.05
        derivative_error = error - self.previous_error
        
        # Limitamos la integral para evitar windup
        self.integral_error = max(-1.0, min(1.0, self.integral_error))
        
        angular_velocity = (self.kp * error + 
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
                    f"Siguiendo pared izq: {self.regions['left']:.2f}m "
                    f"(objetivo: {self.desired_wall_distance:.2f}m)"
                )
                self._last_log_time = time.time()
        else:
            self._last_log_time = time.time()
            
        return cmd

    def search_left_wall(self):
        """Busca una pared en el lado izquierdo"""
        cmd = Twist()
        cmd.linear.x = self.linear_speed * 0.5  # Velocidad reducida durante búsqueda
        cmd.angular.z = self.angular_speed * 0.3  # Giro lento hacia la izquierda
        return cmd

    def stop_robot(self):
        """Detiene el robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        # Guardar este comando como propio
        self.my_last_cmd = cmd
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