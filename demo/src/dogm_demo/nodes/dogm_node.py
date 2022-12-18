#!/usr/bin/env python

import time
import numpy as np
from threading import Lock

from dogm_py import LaserMeasurementGridParams
from dogm_py import LaserMeasurementGrid
from dogm_py import DOGMParams
from dogm_py import DOGM
from dogm_py import VectorFloat
from dogm_py import renderOccupancyGrid

import rospy

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

LIDAR_TOPIC = '/scan'
ODOMETRY_TOPIC = '/odom'
MAP_TOPIC = '/map'


class DynamicOccupancyGrid():

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_time = None

        # DOGM params
        self.grid_size = 20
        self.grid_resolution = 0.1
        particle_count = 20000
        new_born_particle_count = 10000
        persistance_prob = 0.5
        stddev_process_noise_position = 0.1
        stddev_process_noise_velocity = 0.1
        birth_prob = 0.2
        stddev_velocity = 1.0
        init_max_velocity = 5.0
        dogm_params = DOGMParams(size=self.grid_size, resolution=self.grid_resolution, particle_count=particle_count, new_born_particle_count=new_born_particle_count,
                                 persistance_prob=persistance_prob, stddev_process_noise_position=stddev_process_noise_position,
                                 stddev_process_noise_velocity=stddev_process_noise_velocity, birth_prob=birth_prob, stddev_velocity=stddev_velocity,
                                 init_max_velocity=init_max_velocity)
        self.dogm = DOGM(params=dogm_params)

        # DOGM LIDAR params
        fov = 360.0
        angle_increment = 0.0087 * 180.0 / np.pi
        lidar_range = 30.0
        lidar_res = 0.1
        stddev_range = 0.1
        lmg_params = LaserMeasurementGridParams(fov=fov, angle_increment=angle_increment, max_range=lidar_range,
                                                resolution=lidar_res, stddev_range=stddev_range)
        self.lmg = LaserMeasurementGrid(params=lmg_params, size=self.grid_size, resolution=self.grid_resolution)

        # Configure ROS publisher (the map) and subscribers (position and LIDAR)
        self.map_pub = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=1)
        self.lidar_sub = rospy.Subscriber(LIDAR_TOPIC, LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.odom_callback)

        # allocate a mutex/lock
        self.mutex = Lock()

    def lidar_callback(self, data):
        self.mutex.acquire()
        try:
            dt = 0
            now = rospy.get_time()
            if self.last_time is not None:
                dt = now - self.last_time
            self.last_time = now

            grid_data = self.lmg.generateGrid(VectorFloat(data.ranges), self.theta*180.0/np.pi)
            self.dogm.updateGrid(grid_data, self.x, self.y, dt)
        finally:
            self.mutex.release()

    def odom_callback(self, odom):
        self.mutex.acquire()
        try:
            self.x = odom.pose.pose.position.x
            self.y = odom.pose.pose.position.y
            # convert the supplied quaternion into a more convenient Euler angle
            rot = odom.pose.pose.orientation
            _, _, self.theta = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        finally:
            self.mutex.release()

    def publish(self):
        self.mutex.acquire()
        try:

            grid_occupancy = renderOccupancyGrid(self.dogm)

            msg = OccupancyGrid()

            msg.info.map_load_time = rospy.Time.now()
            msg.info.resolution = self.grid_resolution
            msg.info.width = int(self.grid_size / self.grid_resolution)
            msg.info.height = int(self.grid_size / self.grid_resolution)

            # offset the origin to the centre of the map
            msg.info.origin.position.x = self.x
            msg.info.origin.position.y = self.y
            msg.info.origin.position.z = 0

            # ...with zero rotation
            msg.info.origin.orientation.x = 0
            msg.info.origin.orientation.y = 0
            msg.info.origin.orientation.z = 0
            msg.info.origin.orientation.w = 0

            msg.data = np.round(grid_occupancy * 100.).flatten().astype(int).tolist()

            self.map_pub.publish(msg)
        finally:
            self.mutex.release()


def main():
    rospy.init_node('dogm', anonymous=True)

    grid = DynamicOccupancyGrid()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        grid.publish()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
