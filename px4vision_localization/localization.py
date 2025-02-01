#!/usr/bin/env python3

import rclpy
import csv
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import SensorCombined, SensorGps, VehicleLocalPosition
from utilities.frd_to_ned import frdToNed 
from utilities.get_orientation import getOrientation
from utilities.wgs84_ned import wgs84ToNed
from px4vision_localization.ekf import ExtendedKalmanFilter
from std_msgs.msg import String
from px4vision_localization.createPlot import update_ekf_plot, update_vlp_plot

class PX4VisionEKFLocalization(Node):
    def __init__(self):
        super().__init__('px4visionlocalization')
        self.previousTime = None
        self.dt = 0
        initState = np.zeros(9)
        initCov = np.eye(9)
        processNoise = np.diag([1, 1, 3, 0.006, 0.006, 0.006, 0.003, 0.003, 0.003])
        measurementNoise = np.diag([5**2, 5**2, 7**2, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        self.g = 9.81
        self.referenceLat = None
        self.referenceLon = None
        self.referenceAlt = None
        self.R = 6371
        self.vehicleLocalPosition = None


        self.ekf = ExtendedKalmanFilter(initState, initCov, processNoise, measurementNoise)

        #qos profile for publishing and subscribing
        qosProfile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Subscribers
        self.vehicle_imu_subscriber = self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.vehicle_imu_callback, qosProfile)
        self.vehicle_gps_subscriber = self.create_subscription(SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_callback, qosProfile)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qosProfile)

        #Publisher
        self.localization_publisher = self.create_publisher(String, '/ekf_localization', qosProfile)
        self.vehicleLocalPositionPublisher = self.create_publisher(String, '/vehicle_local_position_localization', qosProfile)
        self.create_timer(0.1, self.publish_localization_data)
        self.create_timer(0.1, self.publish_vehicle_local_position)


    def publish_localization_data(self):
        msg = String()
        update_ekf_plot(self.ekf.state[0:3])        
        state_str = np.array2string(self.ekf.state[0:3], precision=4, separator=', ')

        msg.data = f"Localization Data:\n {state_str}"

        print(f"Localization Data:\n {state_str}")

        self.localization_publisher.publish(msg)

    def publish_vehicle_local_position(self):
        msg = String()
        update_vlp_plot(self.vehicleLocalPosition)
        # if self.vehicleLocalPostion is not None:
        #     state_str = f"x: {self.vehicleLocalPosition.x}, y: {self.vehicleLocalPosition.y}, z: {self.vehicleLocalPosition.z}"
        # else:
        #     state_str = "No vehicle local position data available."
        state_str = np.array2string(self.vehicleLocalPosition, precision=4, separator=', ')
        msg.data = f"Vehicle Local Position Data:\n {state_str}"

        print(f"Vehicle Local Position Data:\n {state_str}")

        self.vehicleLocalPositionPublisher.publish(msg)


    def vehicle_imu_callback(self, msg: SensorCombined):
        
        if self.previousTime is not None:
            self.dt = msg.timestamp - self.previousTime
            #self.dt = self.dt / 1e9
            self.dt = 0.0625
            self.previousTime = msg.timestamp
        else:
            self.previousTime = msg.timestamp 
            self.dt = 0.0625

        # imuAcceleration = frdToNed(msg.accelerometer_m_s2[0], msg.accelerometer_m_s2[1], msg.accelerometer_m_s2[2])
        # imuAngularVelocity = frdToNed(msg.gyro_rad[0], msg.gyro_rad[1], msg.gyro_rad[2])
        imuAcceleration = msg.accelerometer_m_s2
        imuAngularVelocity = msg.gyro_rad        
        
        self.ekf.predict(self.dt, imuAcceleration, imuAngularVelocity, self.g)

    def vehicle_gps_callback(self, msg: SensorGps):
        #self.dt = msg.timestamp - self.previousTime
        self.dt = 0.25
        if self.referenceLat is None or self.referenceLon is None or self.referenceAlt is None:
            self.referenceLat = msg.latitude_deg
            self.referenceLon = msg.longitude_deg
            self.referenceAlt = msg.altitude_msl_m

        x, y, z = wgs84ToNed(msg.latitude_deg, msg.longitude_deg, msg.altitude_msl_m, self.referenceLat, self.referenceLon, self.referenceAlt)
        # x = self.R * np.cos(np.deg2rad(msg.latitude_deg)) * np.cos(np.deg2rad(msg.longitude_deg))
        # y = self.R * np.cos(np.deg2rad(msg.latitude_deg)) * np.sin(np.deg2rad(msg.longitude_deg))
        # z = self.R * np.sin(np.deg2rad(msg.latitude_deg))
        vx = msg.vel_n_m_s
        vy = msg.vel_e_m_s
        vz = msg.vel_d_m_s

        gps_measurement = np.array([x, y, z, vx, vy, vz, 0, 0, 0])
        self.ekf.update(gps_measurement) 

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicleLocalPosition = np.array([msg.x, msg.y, msg.z])
    

def main(args=None):
    rclpy.init(args=args)
    px4visionEkfLocalization = PX4VisionEKFLocalization()
    rclpy.spin(px4visionEkfLocalization)
    rclpy.shutdown()          
         