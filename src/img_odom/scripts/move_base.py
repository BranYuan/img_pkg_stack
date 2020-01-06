# _*_ coding: UTF-8 _*_

from math import cos, sin
from wheel import Wheel
import time

class move_base():
    """
    机器人底盘
    """
    def __init__(self):
        self.reducer = 100 * 52.0 / 11
        self.len_chain = 3.7775
        self.left_wheel = Wheel(1, '/dev/ttyS2', self.len_chain, self.reducer)
        self.left_wheel.ini_motor()
        self.right_wheel = Wheel(1, '/dev/ttyS3', self.len_chain,-self.reducer)
        self.right_wheel.ini_motor()
        self.dis_of_2wheels = 1.5

    def write_speed(self, v, v_yaw):
        """
        :param v:
        :param v_yaw:
        :return:
        """
        v_yaw = - v_yaw
        v_r = v + self.dis_of_2wheels * v_yaw / 2
        v_l = v_r - self.dis_of_2wheels * v_yaw
        print v_l, v_r, v_yaw
        self.left_wheel.write_speed(v_l)
        self.right_wheel.write_speed(v_r)
        return True

    def read_speed(self):
        """
        :return: v, v_yaw, 单位： m/s, rad/s
        """
        v_l = self.left_wheel.read_speed()
        v_r = self.right_wheel.read_speed()
        v = (v_r + v_l) / 2
        v_yaw = (v_r - v_l) / self.dis_of_2wheels
        return v, v_yaw

    def read_position(self):
        """
        :return: delta_x, delta_y, delta_theta
        """
        r_dis = self.right_wheel.read_delta_pos()
        l_dis = self.left_wheel.read_delta_pos()
        delta_theta = (r_dis-l_dis) / self.dis_of_2wheels / 2
        if delta_theta != 0:
            radius = (r_dis + l_dis) / (2 * delta_theta)
            delta_x = radius * (1 - cos(delta_theta))
            delta_y = radius * sin(delta_theta)
        else:
            delta_x = r_dis
            delta_y = 0.0
        return delta_x, delta_y, delta_theta


if __name__ == '__main__':
    car = move_base()
    print "####read_speed############"
    print car.read_speed()
    print "####write_speed############"
    # print car.write_speed(0.5, 5 / 360 * 2 * 3.14)
    print car.write_speed(0.0, 0.0 / 180 * 3.14)
    print car.read_speed()
    time.sleep(2)
    # print car.write_speed(0, 0)
    # print "####read_position############"
    # print car.read_position()
    # car.write_speed(-1, 5 / 360 * 2 * 3.14)
    # time.sleep(2)
    # print car.read_position()
    # car.write_speed(-1, -5 / 360 * 2 * 3.14)
    # time.sleep(2)
    # print car.read_position()
    # car.write_speed(1, -5 / 360 * 2 * 3.14)
    # time.sleep(2)
    # print car.read_position()
    # print "####read_speed############"
    # print car.read_speed()
    # print "####write_speed############"
    # print car.write_speed(0, 0)
    # print "####==========############"
