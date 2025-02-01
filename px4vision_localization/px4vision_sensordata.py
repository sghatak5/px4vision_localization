#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import SensorCombined, SensorGps
from utilities.frd_to_ned import frdToNed
from utilities.get_orientation import getOrientation
from utilities.wgs84_ned import wgs84ToNed

class Px4VisionSensorData(Node):
    def __init__(self):
        super().__init__('px4vision_sensordata')
        self.previous_time = None
        self.dt = 0
        self.referenceLat = None
        self.referenceLon = None
        self.referenceAlt = None

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

        if self.referenceLat is None or self.referenceLon is None or self.referenceAlt is None:
            self.referenceLat = msg.latitude_deg
            self.referenceLon = msg.longitude_deg
            self.referenceAlt = msg.altitude_msl_m
            
        x, y, z = wgs84ToNed(msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m, self.referenceLat, self.referenceLon, self.referenceAlt)
        self.get_logger().info(f"GPS Sensor Data:\n Time: {msg.timestamp} \
                                \n X_ned: {x} \
                                \n Y_ned: {y} \
                                \n Z_ned: {z} \
                                \n Velocity_vx (North): {msg.vel_n_m_s} \
                                \n Velocity_vy (East): {msg.vel_e_m_s} \
                                \n Velocity_vz (Down): {msg.vel_d_m_s} ")

    def vehicle_imu_callback(self, msg: SensorCombined):

        if self.previous_time is not None:
            self.dt = msg.timestamp - self.previous_time
            #self.dt = self.dt / 1e9 #Coverting to seconds
            self.previous_time = msg.timestamp
        else:
            self.previous_time = msg.timestamp

        Angular_Velocity = frdToNed(msg.gyro_rad[0], msg.gyro_rad[1], msg.gyro_rad[2])
        Orientation = getOrientation(Angular_Velocity, self.dt)
        self.get_logger().info(f"IMU Sensor Data:\n Time: {msg.timestamp} \
                                \n Angular Velocity (X): {Angular_Velocity[0]} \
                                \n Angular Velocity (Y): {Angular_Velocity[1]} \
                                \n Angular Velocity (Z): {Angular_Velocity[2]} \
                                \n Orientation (X): {Orientation[0]} \
                                \n Orientation (Y): {Orientation[1]} \
                                \n Orientation (Z): {Orientation[2]} ")


def main(args=None):
    rclpy.init(args=args)
    px4vision_sensordata = Px4VisionSensorData()
    rclpy.spin(px4vision_sensordata)
    px4vision_sensordata.destroy_node()
    rclpy.shutdown()