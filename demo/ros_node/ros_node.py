#!/usr/bin/env python

from dogm_py import LaserMeasurementGridParams
from dogm_py import LaserMeasurementGrid
from dogm_py import DOGMParams
from dogm_py import DOGM

import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

LIDAR_TOPIC = '/scan'
ODOMETRY_TOPIO = '/odom'
MAP_TOPIC = '/map'


# DOGM params
grid_size = 20
grid_resolution = 0.1
particle_count = 20000
new_born_particle_count = 10000
persistance_prob = 0.5
stddev_process_noise_position = 0.1
stddev_process_noise_velocity = 0.1
birth_prob = 0.2
stddev_velocity = 1.0
init_max_velocity = 5.0
dogm_params = DOGMParams(size=grid_size, resolution=grid_resolution, particle_count=particle_count, new_born_particle_count=new_born_particle_count,
                         persistance_prob=persistance_prob, stddev_process_noise_position=stddev_process_noise_position,
                         stddev_process_noise_velocity=stddev_process_noise_velocity, birth_prob=birth_prob, stddev_velocity=stddev_velocity,
                         init_max_velocity=init_max_velocity)
dogm_grid = DOGM(params=dogm_params)

# DOGM LIDAR params
fov = 360.0
angle_increment = 0.0087
lidar_range = 20.0
lidar_res = 0.1
stddev_range = 0.1
lmg_params = LaserMeasurementGridParams(fov=fov, angle_increment=angle_increment, max_range=lidar_range, resolution=lidar_res, stddev_range=stddev_range)
lmg = LaserMeasurementGrid(params=lmg_params, size=grid_size, resolution=grid_resolution)


def lidar_callback(data):
    lmg.generateGrid(data.ranges)


def odom_callback(odom):


def main():

    # Configure ROS publisher (the map) and subscribers (position and LIDAR)
    map_pub = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=1)
    lidar_sub = rospy.Subscriber(LIDAR_TOPIC,)

    rospy.init_node('dogm', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
