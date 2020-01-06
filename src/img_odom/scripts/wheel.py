# _*_ coding: UTF-8 _*_

import time
import minimalmodbus
import serial


class Wheel:
    """
    驱动轮类模块
    """
    def __init__(self, id, port, circumference, reducer):
        """
        初始化函数
        :param id: 电机从地址id
        :param port: 电机控制端口号
        :param circumference:轮子每一圈的周长
        :param reducer:电机减速器减速比
        """
        '''
        # 系统状态寄存器地址0x465d: 说明
        '''
        self._addr_data = {
            # 固定速度             编码器线数               电机圈数值            系统状态寄存器：
            'fix_speed': 0x4420, 'rec_line_num': 0x42ea, 'position': 0x42ff, 'sys_status': 0x465d,
            'enable': 0x4657
        }
        self._last_position = 0.0
        self._max_speed = 3000
        self._rec_line_num = 2500
        self._distance_one_circle = 1.0 * circumference / reducer
        self._enable_status = False
        self._last_motor_pos = 0
        self._rec_line_num = 1000
        self._motor = minimalmodbus.Instrument(port, id, debug=False    )
        self._motor.serial.baudrate = 9600

    def ini_motor(self):
        while True:
            try:
                print('init motoring')
                self._rec_line_num = self._motor.read_register(self._addr_data['rec_line_num'])
                break
            except BaseException as e:
                print e
                time.sleep(1)
        self.enable()
        self._motor.read_long(self._addr_data['position'])
        while self.read_position() is None:
            print self.read_position()
            print('initing motor position')
            time.sleep(1)
        return True

    def enable(self):
        """
        电机使能
        :return:
        """
        try:
            self._motor.write_register(self._addr_data['enable'], 1)
            self._enable_status = True
            return True
        except BaseException as e:
            print e
            return False

    def read_speed(self):
        """
        # 读速度，单位:m/s
        :return: speed
        """
        try:
            speed = self._motor.read_register(self._addr_data['fix_speed']) / 10.0
            speed = speed * 60 * self._distance_one_circle
            return speed
        except BaseException as e:
            print e
            return False

    def read_position(self):
        """
        读当前位置，先读取编码器位置，转换为轮子位置，单位：m
        :return: self._last_position, 单位：m
        """
        try:
            self._last_position += self.read_delta_pos()
            return self._last_position
        except BaseException as e:
            print e
            return None

    def read_delta_pos(self):
        """
        读当前位置，先读取编码器位置，转换为轮子位置，单位：m
        :return: delta_dis, 单位：m
        """
        try:
            last_pos = self._last_motor_pos
            self._last_motor_pos = self._motor.read_long(self._addr_data['position'])
            delta_dis = (self._last_motor_pos-last_pos) / self._rec_line_num * self._distance_one_circle
            return delta_dis
        except BaseException as e:
            print e
            return False

    def write_speed(self, speed):
        """
        写速度：单位0.1r/min
        :param speed: 单位：m/s
        :return: bool
        """
        try:
            speed = speed / self._distance_one_circle * 60 * 10
            speed = int(speed)
            speed = speed & 0xffffffff
            print speed
            self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
            return True
        except BaseException as e:
            print e
            return False


if __name__ == '__main__':
    wheel = Wheel(01, '/dev/ttyS2', 3.9, 100)
    print "####enable############"
    print wheel.ini_motor()
    print "####enable############"
    print wheel.enable()
    print "####read_speed############"
    print wheel.read_speed()
    print "####read_position############"
    print wheel.read_position()
    print "####read_delta_pos############"
    print wheel.read_delta_pos()
    print "####write_speed############"
    speed = 2000
    print wheel.write_speed(0.0)
    print "####==========############"
