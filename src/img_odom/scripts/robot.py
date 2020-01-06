# _*_ coding: UTF-8 _*_
import time
from math import sin, cos
from multiprocessing import Queue, Process
import threading
import cv2
import rospy
import tf
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
import gyroscope    # 陀螺仪模块
from qr_code_pose_cam import qr_code_pose_cam     # 视觉里程计模块
# import qr_code_pose_cam     # 视觉里程计模块
from move_base import move_base     # 机器人移动底盘

# debug模式时的里程计积分相关变量
FAKE_X = FAKE_Y = FAKE_TH = 0.0
FAKE_VX = FAKE_V_YAW = 0.0
FAKE_LAST_TIME = time.time()



class robot():
    _debug = False       # debug模式
    _img_odom_flag = False      # 摄像头里程计有效标志
    _x = 0.0        # 里程计x
    _y = 0.0        # 里程计y
    _th = 0.0       # 里程计旋转角
    _th_delta = 0.0     # 里程计旋转角矫正量
    _vx = 0.0       # 速度vx
    _vy = 0.0       # 速度vy
    _vth = 0.0      # 速度vth
    _current_time = 0.0     # 循环监控当前时间
    _last_time = 0.0        # 循环监控上次时间
    _odom_pose_covariance = [[1e-9, 0, 0, 0, 0, 0,
                              0, 1e-3, 1e-9, 0, 0, 0,
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0,
                              0, 0, 0, 0, 1e6, 0,
                              0, 0, 0, 0, 0, 1e-9]]         # 里程计协方差

    def __init__(self, odom_topic='robot'):
        """
        # 机器人初始化，子类：陀螺仪，机器人底盘，里程计发布器，tf广播器，
        :param odom_topic:
        """
        global FAKE_X, FAKE_Y, FAKE_TH, FAKE_LAST_TIME
        self._odom = gyroscope.gyroscope()
        self.move_base = move_base()
        self._odom_pup = rospy.Publisher(odom_topic, Odometry, queue_size=5)
        self._broadcaster = tf.TransformBroadcaster()
        self._current_time = time.time()
        self._last_time = time.time()
        FAKE_LAST_TIME = time.time()
        # while True:
        #     self._th_delta = -self._odom.read_angle_yaw()
        #     print 'initting gyroscope'
        #     if self._th_delta:
        #         break

    def set_img_odom(self, flag):
        """
        # 设置视觉里程计有效标志
        :param flag:
        :return:
        """
        self._img_odom_flag = flag

    def set_debug(self, flag):
        """
        # 开启debug模式
        :param flag:
        :return:
        """
        self._debug = flag

    def reset_odom(self, x, y, th):
        """
        # 矫正里程计当前位置函数
        :param x:
        :param y:
        :param th:
        :return:
        """
        # gy_th = self._odom.read_angle_yaw()
        # if gy_th:
        #     self._th_delta = th - gy_th
        self._x, self._y, self._th = x, y, th
        #print x, y, th

    def spin_once(self, vel, yaw_v):
        """
        # 机器人运行主函数，主要实现：
        # 机器人速度下发
        # 里程计信息获取
        # 里程计TF信息发布
        :param vel:
        :param yaw_v:
        :return:
        """
        # 读取里程计信息
        self._read_odom()
        # 下发机器人速度
        self._write_speed(vel, yaw_v)
        # 里程计TF发布，里程计话题发布
        self.tf_pub()

    def tf_pub(self):
        """
        # TF信息发布
        :return:
        """
        # 里程计tf发布设置
        current_time = rospy.Time.now()
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_footprint"
        # 四元数计算
        quats = tf.transformations.quaternion_about_axis(self._th, (0, 0, 1))
        odom_quat = Quaternion(*quats)
        # tf赋值
        odom_trans.transform.translation.x = self._x
        odom_trans.transform.translation.y = self._y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat
        # tf发布
        self._broadcaster.sendTransformMessage(odom_trans)
        # 里程计话题发布
        msg_odom = Odometry()
        msg_odom.header.stamp = current_time
        msg_odom.header.frame_id = 'odom'
        msg_odom.pose.pose.position.x = self._x
        msg_odom.pose.pose.position.y = self._y
        msg_odom.pose.pose.position.z = 0.0
        msg_odom.pose.pose.orientation = odom_quat
        # msg_odom.pose.covariance = self._odom_pose_covariance
        self._odom_pup.publish(msg_odom)

    def _read_odom(self):
        """
        # 里程计信息获取
        :return:
        """
        if not self._img_odom_flag:
            # 视觉里程计无效时，启用底盘里程计
            # self._vth = self._odom.read_angle_speed_z()     # 读z轴角速度v_yaw
            # th = self._odom.read_angle_yaw()    # 读z轴角度v_yaw
            # self._th = th + self._th_delta      # 角度校正
            delta_x, delta_y, delta_th = self.move_base.read_position()
            self._x += delta_x * cos(self._th)
            self._y += delta_y * sin(self._th)
            self._th += delta_th
            # gy_th = self._odom.read_angle_yaw()
            # if gy_th:
            #     self._th = self._th_delta + gy_th
        if self._debug:
            # debug模式时， 通过订阅vel_cmd话题获取速度并积分计算里程计
            self._x, self._y, self._th = self._get_odom_by_speed()
        else:
            # 非debug模式时，订阅cmd_vel，获取速度信息,并控制机器人运动
            self._get_odom_by_speed()

        print ('position: %.2f,%.2f,%.2f' % (self._x, self._y, self._th / 3.14 * 180))
        return True

    def _write_speed(self, vel, yaw_v):
        self.move_base.write_speed(vel, yaw_v)

    def _get_odom_by_speed(self):
        #########################
        # 订阅'cmd_vel'话题并返回里程计信息
        #########################
        global FAKE_X, FAKE_Y, FAKE_TH, FAKE_LAST_TIME, FAKE_VX, FAKE_V_YAW
        # rospy.Subscriber('cmd_vel', Twist, _callback_speed_topic)
        rospy.Subscriber('cmd_vel', Twist, _callback_speed_topic, callback_args=self)
        x, y, th = FAKE_X, FAKE_Y, FAKE_TH
        return x, y, th

def _callback_speed_topic(data, robot):
    #########################
    # 订阅'cmd_vel'话题的反馈函数
    # 积分计算里程计信息x, y, th
    #########################
    global FAKE_X, FAKE_Y, FAKE_TH, FAKE_LAST_TIME, FAKE_VX, FAKE_V_YAW
    v = data.linear.x
    yaw_v = data.angular.z
    current_time = time.time()
    dt = current_time - FAKE_LAST_TIME
    FAKE_LAST_TIME = current_time
    delta_th = yaw_v * dt
    FAKE_TH += delta_th
    delta = v * dt
    delta_x = delta * cos(FAKE_TH)
    delta_y = delta * sin(FAKE_TH)
    FAKE_X += delta_x
    FAKE_Y += delta_y
    FAKE_VX = v
    FAKE_V_YAW = yaw_v
    # robot._write_speed(v, yaw_v)


#########################
# 模块共用函数
#########################
def _reverse_tf(tanslation, euler):
    '''
    # 输入二维码在相机坐标系下的坐标，输出相机base_footprint在二维码坐标系下的坐标
    :param tanslation:
    :param euler:
    :return: t, euler_angle
    '''
    transform = tf.Transformer(True)
    m = TransformStamped()
    m.header.frame_id = 'camera'
    m.child_frame_id = 'qr_code'
    m.transform.translation.x = tanslation[0]
    m.transform.translation.y = tanslation[1]
    m.transform.translation.z = tanslation[2]
    quaternion = tf.transformations.quaternion_from_euler(*euler)
    m.transform.rotation.x = quaternion[0]
    m.transform.rotation.y = quaternion[1]
    m.transform.rotation.z = quaternion[2]
    m.transform.rotation.w = quaternion[3]
    transform.setTransform(m)

    m.header.frame_id = 'base_footprint'
    m.child_frame_id = 'camera'
    m.transform.translation.x = 0.88
    m.transform.translation.y = 0.72
    m.transform.translation.z = 0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, -1.57)
    m.transform.rotation.x = quaternion[0]
    m.transform.rotation.y = quaternion[1]
    m.transform.rotation.z = quaternion[2]
    m.transform.rotation.w = quaternion[3]
    transform.setTransform(m)
    t, q = transform.lookupTransform('qr_code', 'base_footprint', rospy.Time(0))
    euler_angle = tf.transformations.euler_from_quaternion(q)
    return t, euler_angle


def img_odom_loop(que):
    # 视觉里程计进程
    pre_time = time.time()
    odom_cam = qr_code_pose_cam()
    # odom_cam = qr_code_pose_cam.qr_code_pose_cam()
    while not rospy.is_shutdown():
        frame, depth_data = odom_cam.get_img()  # 获取一帧图像
        qr_datas, qr_four_conners = odom_cam.qr_decode(frame)
        x0, y0, th0 = 0.0, 0.0, 0.0
        flag = False
        if qr_datas:
            for i in range(len(qr_four_conners)):
                qr_name = qr_datas[i]
                if qr_name == '00000000000':
                    print qr_name
                    x0, y0, th0 = -0.046, 3.825, 0.0
                    flag = True
                elif qr_name == '00000000001':
                    x0, y0, th0 = -0.040, 35.975, 0.0
                    flag = True
                elif qr_name == '00000000002':
                    x0, y0, th0 = 0.040, 36.025, 3.14
                    flag = True
                elif qr_name == '00000000003':
                    x0, y0, th0 = 0.038, 4.025, 3.14
                    flag = True
                else:
                    flag = False
                if flag:
                    print qr_four_conners
                    position = odom_cam.get_qrcode_pose(qr_four_conners[i], depth_data)
                    print position
                    th = position[1]
                    x = position[0][0] / 1000
                    y = position[0][2] / 1000
                    z = position[0][1] / 1000
                    if th:
                        th = th / 180 * 3.14
                        (x, y, z), (angle_x, angle_y, th) =_reverse_tf((x, y, z), (0, 0, th))
                        pose = (x0+x-0.5, y0+y, th0+th)
                        print '*************************************'
                        print x, y, z, th
                        if not que.full():
                            que.put_nowait(pose)
                        else:
                            try:
                                que.get_nowait()
                                que.put_nowait(pose)
                            except BaseException, e:
                                print e
        print time.time() - pre_time
        pre_time = time.time()
        cv2.imshow("Video", frame)
        if cv2.waitKey(1) == 27:
            break
    if not que.full():
        que.put(1)
    else:
        try:
            que.get_nowait()
            que.put(1)
        except BaseException, e:
            print e
    odom_cam.release()
    cv2.destroyAllWindows()

def robot_loop(que):
    # 机器人进程
    global FAKE_X
    global FAKE_Y
    global FAKE_TH
    global robot
    robot = robot()
    robot.set_debug(False)
    robot.set_img_odom(False)
    while not rospy.is_shutdown():
        if not que.empty():
            pose = que.get_nowait()
            if pose[-1]:
                robot.set_img_odom(True)
                # robot.set_debug(False)
                x, y, th = pose[0], pose[1], pose[2]
                robot.reset_odom(x, y, th)
                FAKE_X, FAKE_Y, FAKE_TH = x, y, th
                # print '==================================='
        else:
            robot.set_img_odom(False)
            # robot.set_debug(True)
        # time.sleep(0.1)
        robot.spin_once(0, 0)

def main_loop():
    rospy.init_node('robot')
    que = Queue(2)
    img_odom_process = Process(target=img_odom_loop, args=(que,))
    robot_process = Process(target=robot_loop, args=(que,))
    img_odom_process.start()
    robot_process.start()
    robot_process.join()
    img_odom_process.join()


if __name__ == '__main__':
    rospy.init_node('robot')
    robot = robot()
    robot.set_debug(True)
    robot.set_img_odom(False)
    que = Queue(2)

    img_odom_process = Process(target=img_odom_loop, args=(que,))
    img_odom_process.start()

    while not rospy.is_shutdown():
        print 'lichengji ++++++++++++++++++++++++++++++'
        if not que.empty():
            pose = que.get_nowait()
            if pose == 1:
                break
            if pose[-1]:
                robot.set_img_odom(True)
                x, y, th = pose[0], pose[1], pose[2]
                robot.reset_odom(x, y, th)
                FAKE_X, FAKE_Y, FAKE_TH = x, y, th
                # print '==================================='
        else:
            robot.set_img_odom(False)
        robot.spin_once(FAKE_VX, FAKE_V_YAW)
        time.sleep(0.2)

    # img_odom_process.join()

