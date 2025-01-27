#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import SensorCombined, SensorGps

class Px4VisionSensorData(Node):
    def __init__(self):
        super().__init__('px4vision_sensordata')

        #qos profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Subscribers
        self.vehicle_gps_subscriber = self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qos_profile)
        self.vehicle_imu_subscriber = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.vehicle_imu_callback, qos_profile)


    def vehicle_gps_callback(self, msg: SensorGps):
        self.get_logger().info(f"GPS Sensor Data:\n Time: {msg.timestamp} \
                                \n Latitude: {msg.latitude_deg} \
                                \n Longitude: {msg.longitude_deg} \
                                \n Altitude: {msg.altitude_msl_m} \
                                \n Velocity_vx (North): {msg.vel_n_m_s} \
                                \n Velocity_vy (East): {msg.vel_e_m_s} \
                                \n Velocity_vz (Down): {msg.vel_d_m_s} ")

    def vehicle_imu_callback(self, msg: SensorCombined):
        self.get_logger().info(f"IMU Sensor Data:\n Time: {msg.timestamp} \
                                \n Angular Velocity (X): {msg.gyro_rad[0]} \
                                \n Angular Velocity (Y): {msg.gyro_rad[1]} \
                                \n Angular Velocity (Z): {msg.gyro_rad[2]}")



def main(args=None):
    rclpy.init(args=args)
    px4vision_sensordata = Px4VisionSensorData()
    rclpy.spin(px4vision_sensordata)
    px4vision_sensordata.destroy_node()
    rclpy.shutdown()