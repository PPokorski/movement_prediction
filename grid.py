import math

import rospy
import nav_msgs.msg
import sensor_msgs.msg

time_step = 0.1

good_hit = 0.99
good_miss = 0.99

max_probability = 0.99
min_probability = 0.01

default_value = 0.5

min_velocity = 0.1
max_velocity = 0.5
number_of_velocities = 3

base_laser_link = 'base_laser_link'
zero_point = [0.0, 0.0]


def sign(value):
    return int(math.copysign(1, value))


class OccupancyGrid:
    def __init__(self, width_m, height_m, resolution_mpp):

        self.width_m = width_m
        self.height_m = height_m
        self.resolution_mpp = resolution_mpp

        self.width_p = int(round(width_m / resolution_mpp))
        self.height_p = int(round(height_m / resolution_mpp))

        self.origin = [-self.width_m / 2.0, -self.height_m / 2.0]

        self.map = [[default_value for column in range(self.width_p)] for row in range(self.height_p)]

        self.map_publisher = rospy.Publisher('occupancy_map', nav_msgs.msg.OccupancyGrid, queue_size=1)
        self.laserscan_subscriber = rospy.Subscriber('base_scan', sensor_msgs.msg.LaserScan, self.laserscan_callback, queue_size=1)

    def publish_occupancy_grid(self):
        msg = nav_msgs.msg.OccupancyGrid()

        msg.header.frame_id = base_laser_link
        msg.header.stamp = rospy.Time.now()

        msg.info.resolution = self.resolution_mpp
        msg.info.width = self.width_p
        msg.info.height = self.height_p
        msg.info.origin.position.x = self.origin[0]
        msg.info.origin.position.y = self.origin[1]
        msg.info.origin.orientation.w = 1.0

        msg.data = [0 for i in range(self.width_p * self.height_p)]

        for row in range(self.height_p):
            for column in range(self.width_p):
                msg.data[column * self.height_p + row] = int(self.map[row][column] * 128)

        self.map_publisher.publish(msg)

    def indices_to_position(self, indices):
        return [indices[0] * self.resolution_mpp + self.origin[0],
                indices[1] * self.resolution_mpp + self.origin[1]]

    def position_to_indices(self, position):
        return [int(round((position[0] - self.origin[0]) / self.resolution_mpp)),
                int(round((position[1] - self.origin[1]) / self.resolution_mpp))]

    def is_in_map_indices(self, indices):
        return not (indices[0] < 0 or indices[0] > self.height_p - 1 or indices[1] < 0 or indices[1] > self.width_p - 1)

    def is_in_map_position(self, position):
        return self.is_in_map_indices(self.position_to_indices(position))

    def bayes_rule(self, p_a_if_b, p_b):
        value = p_a_if_b * p_b / (p_a_if_b * p_b + (1 - p_a_if_b) * (1 - p_b))
        value = max(value, min_probability)
        value = min(value, max_probability)
        return value

    def mark_value(self, p_occupancy):
        return self.bayes_rule(good_hit, p_occupancy)

    def free_value(self, p_occupancy):
        return 1.0 - (self.bayes_rule(good_miss, 1.0 - p_occupancy))

    # https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    def ray_trace(self, start_point, end_point):
        if start_point == end_point:
            # self.try_mark_cell(grid, start_point)
            return

        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        sign_x = sign(dx)
        sign_y = sign(dy)
        dx = abs(dx)
        dy = abs(dy)
        x = start_point[0]
        y = start_point[1]

        change = False

        if dy > dx:
            dy, dx = dx, dy
            change = True

        e = 2 * dy - dx
        i = 1
        while i <= dx:

            if not self.is_in_map_indices([x, y]):
                return

            self.map[x][y] = self.free_value(self.map[x][y])

            if e >= 0:
                if not change:
                    y += sign_y
                else:
                    x += sign_x
                e -= 2 * dx

            if e < 0:
                if not change:
                    x += sign_x
                else:
                    y += sign_y
                e += 2 * dy

            i += 1

    def mark_cell(self, position):

        [x, y] = self.position_to_indices(position)

        self.map[x][y] = self.mark_value(self.map[x][y])

    def laserscan_callback(self, data):

        for i in range(len(data.ranges)):
            ray_angle = data.angle_min + i * data.angle_increment
            ray_range = data.ranges[i]

            x = ray_range * math.cos(ray_angle)
            y = ray_range * math.sin(ray_angle)

            self.ray_trace(self.position_to_indices(zero_point),
                           self.position_to_indices([x, y]))

            if self.is_in_map_position([x, y]):
                self.mark_cell([x, y])




if __name__ == '__main__':

    rospy.init_node('occupancy_grid')

    grid = OccupancyGrid(1.0, 1.0, 0.05)

    rate = rospy.Rate(1 / time_step)
    try:

        while not rospy.is_shutdown():
            grid.publish_occupancy_grid()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
