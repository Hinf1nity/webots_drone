import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np
from controller import Supervisor
from time import sleep
from tf2_ros import TransformBroadcaster, TransformStamped


class DroneDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Devices
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__devices = {
            'camera_left': self.__robot.getDevice('camera_left'),
            'camera_right': self.__robot.getDevice('camera_right'),
            'lidar': self.__robot.getDevice('lidar_livox'),
            'imu': self.__robot.getDevice('imu_livox')
        }

        # 1. Inicializar los 4 motores
        self.__motors = {
            'front_left':  self.__robot.getDevice('motor_propeller_front_left'),
            # *-1
            'front_right': self.__robot.getDevice('motor_propeller_front_right'),
            'rear_left':   self.__robot.getDevice('motor_propeller_rear_left'),
            # *-1
            'rear_right':  self.__robot.getDevice('motor_propeller_rear_right')
        }

        # Configurar motores para control por velocidad
        for motor in self.__motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        sleep(1)  # Esperar un momento para que los motores se estabilicen

        # 2. Sensores (activar los que se necesiten)
        # self.__devices['camera_left'].enable(self.__timestep)
        # self.__devices['camera_right'].enable(self.__timestep)
        # self.__devices['lidar'].enable(self.__timestep)

        # 4. Comando de velocidad objetivo
        self.__target_twist = Twist()

        # 6. Nodo ROS 2
        rclpy.init(args=None)
        self.__node = rclpy.create_node('thaiquiri_driver')

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
