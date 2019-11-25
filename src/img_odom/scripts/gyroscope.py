# _*_ coding: UTF-8 _*_
"""
# 维特智能陀螺仪模块驱动程序
# 型号：WT61PC485，六轴陀螺仪，485通信。MODBUS RTU协议
# 输出数据内容
"""
import minimalmodbus as modbus


class gyroscope:
    def __init__(self, port='/dev/ttyS4', slave_address=0x50, baudrate=9600):
        ###################################
        # 485通信设置
        ###################################
        self.slave = modbus.Instrument(port, slave_address)
        # self.slave.debug = True
        self.slave.serial.baudrate = baudrate
        self.slave.serial.timeout = 0.2
        self.slave.serial.write_timeout = 0.2
        ###################################
        # 寄存器地址
        ###################################
        # 加速度地址
        self._addr_ax = 0x34
        self._addr_ay = 0x35
        self._addr_az = 0x36
        # 角速度地址
        self._addr_wx = 0x37
        self._addr_wy = 0x38
        self._addr_wz = 0x39
        # 角度地址
        self._addr_roll = 0x3d
        self._addr_pitch = 0x3e
        self._addr_yaw = 0x3f
        self._addr_tempture = 0x40
        # 四元数开始地址
        self._addr_quaternion = 0x51
        ###################################
        # 输出转换公式
        ###################################
        self._g = 9.8   # 重力加速度
        self._t_a = 1.0 / 32768 * 16 * self._g     # 加速度转换式，单位：m/s2
        self._t_w = 1.0 / 32768 * 2000 * 3.1416 / 180       # 角速度转换式 单位: rad/s
        self._t_angle = 1.0 / 32768 * 3.1416    # 角度转换式，单位： rad
        self._t_q = 1.0 / 32768     # 四元数转换式

    ###################################
    # 读取目标寄存器值方法
    ###################################
    def _read_goals(self, addr, num, transfer=1.0):
        # 读取多个目标地址的值，并根据转换关系转化
        try:
            goal_values = self.slave.read_registers(addr, num)
            for i in range(num):
                goal_values[i] *= transfer
            return goal_values
        except:
            return False

    def _read_goal(self, addr, transfer=1.0):
        # 读取多个目标地址的值，并根据转换关系转化
        try:
            goal_value = self.slave.read_register(addr)
            goal_value *= transfer
            return goal_value
        except:
            return False

    ###################################
    # 读取加速度方法
    ###################################
    def read_accelerates(self):
        # 读取加速度值
        return self._read_goals(self._addr_ax, 3, self._t_a)

    def read_accelerate_x(self):
        # 读取x轴加速度值
        return self._read_goal(self._addr_ax, self._t_a)

    def read_accelerate_y(self):
        # 读取y轴加速度值
        return self._read_goal(self._addr_ay, self._t_a)

    def read_accelerate_z(self):
        # 读取z轴加速度值
        return self._read_goal(self._addr_az, self._t_a)

    ###################################
    # 读取角速度方法
    ###################################
    def read_angle_speeds(self):
        # 读取加速度值
        return self._read_goals(self._addr_wx, 3, self._t_w)

    def read_angle_speed_x(self):
        # 读取x轴角速度值
        return self._read_goal(self._addr_wx, self._t_w)

    def read_angle_speed_y(self):
        # 读取y轴角速度值
        return self._read_goal(self._addr_wy, self._t_w)

    def read_angle_speed_z(self):
        # 读取z轴角速度值
        return self._read_goal(self._addr_wz, self._t_w)

    ###################################
    # 读取角度方法, 欧拉角坐标系旋转顺序:z--->y--->x
    ###################################
    def read_angles(self):
        # 读取角速度值
        return self._read_goals(self._addr_roll, 3, self._t_angle)

    def read_angle_roll(self):
        # 读取x轴(roll)角速度值
        return self._read_goal(self._addr_roll, self._t_angle)

    def read_angle_pitch(self):
        # 读取y轴(pitch)角速度值
        return self._read_goal(self._addr_pitch, self._t_angle)

    def read_angle_yaw(self):
        # 读取z轴(yaw)角速度值
        return self._read_goal(self._addr_yaw, self._t_angle)

    ###################################
    # 读取四元数方法
    ###################################
    def read_quaternion(self):
        # 读取四元数值
        return self._read_goals(self._addr_quaternion, 4, self._t_q)

if __name__ == "__main__":
    gyroscope = gyroscope()
    print "accelerates"
    print gyroscope.read_accelerates()
    print gyroscope.read_accelerate_x()
    print gyroscope.read_accelerate_y()
    print gyroscope.read_accelerate_z()
    print "angle speeds"
    print gyroscope.read_angle_speeds()
    print gyroscope.read_angle_speed_x()
    print gyroscope.read_angle_speed_y()
    print gyroscope.read_angle_speed_z()
    print "angels"
    print gyroscope.read_angles()
    print gyroscope.read_angle_roll()
    print gyroscope.read_angle_pitch()
    print gyroscope.read_angle_yaw()
    print "quaternion"
    print gyroscope.read_quaternion()