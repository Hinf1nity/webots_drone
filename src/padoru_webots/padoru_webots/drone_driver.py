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
        # self.supervisor = Supervisor()
        self.__robot = webots_node.robot
        # self.robot_node = self.supervisor.getFromDef("robot")
        # self.trans_field = self.robot_node.getField("translation")
        # self.rot_field = self.robot_node.getField("rotation")

        # Devices
        self.__timestep = int(self.__robot.getBasicTimeStep())
        self.__devices = {
            'camera': self.__robot.getDevice('camera'),
            'camera_roll': self.__robot.getDevice('camera roll'),
            'camera_pitch': self.__robot.getDevice('camera pitch'),
            'lidar': self.__robot.getDevice('lidar'),
            'gps': self.__robot.getDevice('gps'),
            'imu': self.__robot.getDevice('inertial unit'),
            'compass': self.__robot.getDevice('compass'),
            'gyro': self.__robot.getDevice('gyro'),
            'front_left_led': self.__robot.getDevice('front left led'),
            'front_right_led': self.__robot.getDevice('front right led'),
        }

        # 1. Inicializar los 4 motores
        self.__motors = {
            'front_left':  self.__robot.getDevice('front left propeller'),
            'front_right': self.__robot.getDevice('front right propeller'),
            'rear_left':   self.__robot.getDevice('rear left propeller'),
            'rear_right':  self.__robot.getDevice('rear right propeller')
        }

        # Configurar motores para control por velocidad
        for motor in self.__motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)

        sleep(1)  # Esperar un momento para que los motores se estabilicen

        # 2. Sensores (activar los que se necesiten)
        self.__devices['camera'].enable(self.__timestep)
        self.__devices['lidar'].enable(self.__timestep)
        self.__devices['gps'].enable(self.__timestep)
        self.__devices['imu'].enable(self.__timestep)
        self.__devices['gyro'].enable(self.__timestep)

        # 3. Control PID
        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6
        self.k_vertical_p = 3.0
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0
        self.target_altitude = 0.5

        # 4. Comando de velocidad objetivo
        self.__target_twist = Twist()

        # 5. Filtro de Kalman
        self.state = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.P = np.eye(6) * 0.1  # Matriz de covarianza inicial
        # Matriz de covarianza del proceso (Que tanto confiamos en nuestro modelo)
        self.Q = np.eye(6) * 0.01
        # Matriz de covarianza de la medición (Que tanto confiamos en nuestros sensores)
        self.R = np.eye(6) * 0.05

        # 6. Nodo ROS 2
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')
        self.__node.create_subscription(
            Twist, 'cmd_pos', self.__cmd_vel_callback, 1)
        self.__odometry_publisher = self.__node.create_publisher(
            Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self.__node)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def clamp(self, n, min_n, max_n):
        return max(min_n, min(n, max_n))

    def kalman_state(self, gps_data, imu_angles):
        """
        Implementación simplificada de un filtro de Kalman para estimar la posición y orientación del dron.
        Input:
        gps_data: [x, y, z] de getValues()
        imu_angles: [roll, pitch, yaw] de getRollPitchYaw()
        Output:
        state: [x, y, z, roll, pitch, yaw]
        """
        # 1. PREDICCIÓN (Basada en modelo simple o datos previos)
        # En un dron, la IMU suele dictar la orientación instantánea
        self.state[3:6] = imu_angles

        # 2. MEDICIÓN (Combinamos GPS con los ángulos)
        z = np.array([gps_data[0], gps_data[1], gps_data[2],
                      imu_angles[0], imu_angles[1], imu_angles[2]])

        # 3. ACTUALIZACIÓN (Filtro de Kalman simplificado)
        # Calculamos la ganancia de Kalman (K)
        H = np.eye(6)  # Matriz de observación
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Actualizamos el estado con la nueva medición
        y = z - (H @ self.state)  # Innovación
        self.state = self.state + K @ y

        # Actualizamos la incertidumbre
        self.P = (np.eye(6) - K @ H) @ self.P + self.Q

        return self.state

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        roll = self.__devices['imu'].getRollPitchYaw()[0]
        pitch = self.__devices['imu'].getRollPitchYaw()[1]
        altitude = self.__devices['gps'].getValues()[2]
        roll_acceleration = self.__devices['gyro'].getValues()[0]
        pitch_acceleration = self.__devices['gyro'].getValues()[1]

        self.__devices['camera_roll'].setPosition(-0.115 * roll_acceleration)
        self.__devices['camera_pitch'].setPosition(-0.1 * pitch_acceleration)

        self.__devices['front_left_led'].set(
            1 if self.__target_twist.linear.x > 0 else 0)
        self.__devices['front_right_led'].set(
            1 if self.__target_twist.linear.x > 0 else 0)

        pitch_disturbance = self.__target_twist.linear.x
        roll_disturbance = self.__target_twist.linear.y
        self.target_altitude += self.__target_twist.linear.z
        yaw_disturbance = self.__target_twist.angular.z
        self.__target_twist.linear.z = 0.0
        self.__target_twist.angular.z = self.__target_twist.angular.z * \
            0.7 if abs(self.__target_twist.angular.z) > 0 else 0

        roll_input = self.k_roll_p * \
            self.clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * \
            self.clamp(pitch, -1.0, 1.0) + \
            pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = self.clamp(
            self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * \
            pow(clamped_difference_altitude, 3.0)

        front_left_motor_input = self.k_vertical_thrust + \
            vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = self.k_vertical_thrust + \
            vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = self.k_vertical_thrust + \
            vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = self.k_vertical_thrust + \
            vertical_input + roll_input - pitch_input - yaw_input

        self.__motors['front_left'].setVelocity(front_left_motor_input)
        self.__motors['front_right'].setVelocity(-front_right_motor_input)
        self.__motors['rear_left'].setVelocity(-rear_left_motor_input)
        self.__motors['rear_right'].setVelocity(rear_right_motor_input)

        # trans = self.trans_field.getSFVec3f()
        # rot = self.rot_field.getSFRotation()

        # state = self.kalman_state(
        #     self.__devices['gps'].getValues(), self.__devices['imu'].getRollPitchYaw())
        trans = self.__devices['gps'].getValues()
        rot = self.__devices['imu'].getRollPitchYaw()
        r = R.from_euler('xyz', [rot[0], rot[1], rot[2]], degrees=False)

        odometry_msg = Odometry()

        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_link'
        odometry_msg.header.stamp = self.__node.get_clock().now().to_msg()

        odometry_msg.pose.pose.position.x = trans[0]
        odometry_msg.pose.pose.position.y = trans[1]
        odometry_msg.pose.pose.position.z = trans[2]

        odometry_msg.pose.pose.orientation.w = r.as_quat()[3]
        odometry_msg.pose.pose.orientation.x = r.as_quat()[0]
        odometry_msg.pose.pose.orientation.y = r.as_quat()[1]
        odometry_msg.pose.pose.orientation.z = r.as_quat()[2]

        self.__odometry_publisher.publish(odometry_msg)

        # TF transform
        t = TransformStamped()

        t.header.stamp = self.__node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]

        t.transform.rotation.x = r.as_quat()[0]
        t.transform.rotation.y = r.as_quat()[1]
        t.transform.rotation.z = r.as_quat()[2]
        t.transform.rotation.w = r.as_quat()[3]

        self.tf_broadcaster.sendTransform(t)
