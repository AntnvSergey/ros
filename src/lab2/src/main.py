#! /usr/bin/python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData


class Occupancy_grid:
    def __init__(self):
        self.grid = 0
        self.grid_size = 300
        self.grid_center = [150, 150]
        self.scale = self.grid_size / 10

    def draw_points(self, ranges, angles, data):

        for i, laser_data in enumerate(ranges):

            x = laser_data * np.sin(angles[i]) * self.scale + self.grid_center[0]
            y = laser_data * np.cos(angles[i]) * self.scale + self.grid_center[1]

            if abs(x) < data.shape[0] and abs(y) < data.shape[1]:
                data[abs(int(x))][abs(int(y))] = 200

                coord = np.array([abs(int(x)), abs(int(y))])
                delta = coord - np.array(self.grid_center)
                dist = np.linalg.norm(delta)
                route = delta / np.linalg.norm(delta)

                if dist < np.inf:
                    for i in range(int(dist)):
                        point = np.array(self.grid_center + route * i).astype(int)
                        if point[0] < data.shape[0] and point[1] < data.shape[1]:
                            data[point[0]][point[1]] = 0
                        else:
                            break

    def create_grid(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        angles = np.arange(angle_min, angle_max, angle_increment)

        data = np.full((self.grid_size, self.grid_size), fill_value=-1)
        self.draw_points(ranges, angles, data)

        self.grid = np.flipud(data).flatten().astype(np.int8).tolist()


if __name__ == "__main__":
    rospy.init_node('occupancy_grid_creator')
    rate = rospy.Rate(1)
    Grid = Occupancy_grid()

    publisher = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=1)
    subscriber = rospy.Subscriber('/base_scan', LaserScan, Grid.create_grid)

    while not rospy.is_shutdown():
        if Grid.grid:
            meta_data = MapMetaData()
            meta_data.resolution = 1 / Grid.scale

            meta_data.width = Grid.grid_size
            meta_data.height = Grid.grid_size

            map_msg = OccupancyGrid()
            map_msg.data = Grid.grid
            map_msg.header.frame_id = '/base_laser_link'

            pos = np.array([0, 0, 0])
            meta_data.origin = Pose()
            meta_data.origin.position.x = pos[0]
            meta_data.origin.position.y = pos[1]
            map_msg.info = meta_data

            rospy.loginfo('publish created grid')
            publisher.publish(map_msg)
        rate.sleep()
