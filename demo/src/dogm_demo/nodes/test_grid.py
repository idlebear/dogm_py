#!/usr/bin/env python

from threading import Lock

from dogm_py import LaserMeasurementGridParams
from dogm_py import LaserMeasurementGrid
from dogm_py import DOGMParams
from dogm_py import DOGM
from dogm_py import renderOccupancyGrid


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
        angle_increment = 0.0087
        lidar_range = 20.0
        lidar_res = 0.1
        stddev_range = 0.1
        lmg_params = LaserMeasurementGridParams(fov=fov, angle_increment=angle_increment, max_range=lidar_range,
                                                resolution=lidar_res, stddev_range=stddev_range)
        self.lmg = LaserMeasurementGrid(params=lmg_params, size=self.grid_size, resolution=self.grid_resolution)


def main():
    grid = DynamicOccupancyGrid()

    occ = renderOccupancyGrid(grid.dogm)

    cell_data = grid.dogm.getGridCells()

    free = cell_data.get_free_mass()
    print(free.shape)


if __name__ == '__main__':
    main()
