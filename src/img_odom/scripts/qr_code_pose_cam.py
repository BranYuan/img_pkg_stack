# _*_ coding: UTF-8 _*_

import time
import copy
import numpy
import pyzbar.pyzbar as pyzbar
import cv2
from openni import openni2
from openni import _openni2 as c_api
from math import atan2


class qr_code_pose_cam:
    # 输入二维矩阵：
    # 第一维是要求解的二维码列表，第二维是对应二维码四个角点坐标
    # 求解并返回二维码位姿
    _tag_size = 199.0
    _tag_size_angle = pow(pow(_tag_size, 2) * 2, 0.5)
    # 相机内参
    _fx = 1357.612626
    _fy = 1350.212414
    _cx = 348.558447
    _cy = 236.904019
    # 相机内参矩阵
    _camera_matrix = numpy.array([[_fx, 0, _cx],
                                 [0, _fy, _cy],
                                 [0, 0, 1]], dtype=numpy.float64)
    # 相机畸变矩阵
    _dist_coeffs = numpy.array([-0.342694, -0.456201, -0.000859, 0.001472, 0.0], dtype=numpy.float64)
    # 二维码坐标轴长度矩阵
    _axis = numpy.array([[_tag_size / 2, 0, 0],
                        [0, _tag_size / 2, 0],
                        [0, 0, _tag_size / 2]],
                        dtype=numpy.float32)
    # 二维码四角点的3d坐标
    # _obj_points = numpy.array([[0, 0, 0],
    #                            [0, _tag_size, 0],
    #                            [_tag_size, 0, 0],
    #                            [_tag_size, _tag_size, 0],
    #                            [0, _tag_size/3, 0],
    #                            [_tag_size/3, 0, 0],
    #                            [0, _tag_size/3*2, 0],
    #                            [_tag_size/3*2, 0, 0],
    #                            [_tag_size/3, _tag_size/3, 0]
    #                            ], dtype=numpy.float64)
    _obj_points = numpy.array([[0, 0, 0],
                               [0, _tag_size, 0],
                               [0, 0, -_tag_size],
                               [0, _tag_size, -_tag_size],
                               ], dtype=numpy.float64)

    def __init__(self, cam_id="rtsp://admin:@192.168.52.52/h265/main/av_stream"):
        self._cam_id = 0  # cam_id
        self._fps = 30
        try:
            self._cap = cv2.VideoCapture(self._cam_id)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)
            self._height = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self._width = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self._cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            print self._cap.get(cv2.CAP_PROP_BRIGHTNESS)
            print self._cap.set(cv2.CAP_PROP_BACKLIGHT, -64)
            openni2.initialize()
            self.device = openni2.Device.open_any()
            print 'initialized'
            self.depth_stream = self.device.create_depth_stream()
            self.depth_stream.set_mirroring_enabled(False)
            self.corlor_stream = self.device.create_color_stream()
            self.corlor_stream.set_mirroring_enabled(False)
            self.device.set_depth_color_sync_enabled(True)
            self.depth_stream.start()
            self.corlor_stream.start()
            self.depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=640, resolutionY=480, fps=self._fps))
            # self.corlor_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX=640, resolutionY=480, fps=1))
        except Exception as e:
            print e

    def set_cam_id(self, cam_id):
        # 设置摄像头设备号
        try:
            cap = cv2.VideoCapture(self._cam_id)
            self._cap = cap
            self._cam_id = cam_id
            return True
        except Exception as e:
            print e
            return False

    def release(self):
        self._cap.release()

    def get_img(self):
        try:
            # 分辨率1080P, 码率8192，帧率1，缩放0.5
            (ret, frame) = self._cap.read()  # 获取一帧图像
            depth_frame = self.depth_stream.read_frame()
            frame_data = depth_frame.get_buffer_as_uint16()
            # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)  # 改变图像大小
            return frame, frame_data
        except Exception as e:
            print e
            return None

    def qr_decode(self, image):
        barcodes = pyzbar.decode(image)
        edges = cv2.Canny(image, 100, 120)
        # cv2.imshow('edges', edges)
        datas = []
        four_conners = []
        if not barcodes:
            return None, None
        for barcode in barcodes:
            barcode_data = barcode.data.decode("utf-8")  # 解码data
            barcode_type = barcode.type
            text = "{}".format(barcode_data)
            points = barcode.polygon
            points = numpy.array([[i.x, i.y] for i in points])
            cv2.polylines(image, [points.copy()], True, (0, 0, 255), 2)  # 画二维码框
            points = self._sort_points(points)
            first = (points[0][0], points[0][1])
            second = (points[1][0], points[1][1])
            cv2.circle(image, first, 5, (255, 0, 0))
            cv2.circle(image, second, 5, (0, 255, 0))
            x = points[0][0]
            y = points[0][1]
            cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 125), 2)
            datas.append(barcode_data)
            four_conners.append(points)
        return datas, four_conners

    def get_qrcode_pose(self, four_coner, depth_data, qr_tolerance=20):
        # 求解二维码位姿
        # 第一角点世界坐标
        axis = [self._tag_size, 0]
        x = four_coner[0][0]
        y = four_coner[0][1]
        index = int(y * self._width + x)
        z = depth_data[index]
        position_i = openni2.convert_depth_to_world(self.depth_stream, x, y, z)
        # 第二角点世界坐标
        x = four_coner[1][0]
        y = four_coner[1][1]
        index = int(y * self._width + x)
        z = depth_data[index]
        position_j = openni2.convert_depth_to_world(self.depth_stream, x, y, z)
        # 两点距离
        length = self._calcu_distence(position_i, position_j)
        print 'qr_code length is %d' % length
        print x, y, z, depth_data[index]
        if abs(length - self._tag_size) < qr_tolerance:
            v1_2 = (position_j[1]-position_i[1], position_j[2] - position_i[2])
            dx = v1_2[0] - axis[0]
            dz = v1_2[1] - axis[1]
            angle = atan2(-dz, -dx)
            # if angle >= 0:
            angle = -angle/3.14*180
        else:
            angle = None
        position = (position_i, angle)
        # print position
        return position

    def _min_tolerance_point(self, four_coner, depth_data, qr_tolerance):
        # 寻找误差最小的点
        position = []
        delta_distence = []
        index = []
        count = len(four_coner)
        for coner in four_coner:
            xi = coner[0]
            yi = coner[1]
            index_i = int(yi * self._width + xi)
            zi = depth_data[index_i]
            position.append(openni2.convert_depth_to_world(self.depth_stream, xi, yi, zi))
        for i in range(count - 1):
            for j in range(i + 1, count):
                length = self._calcu_distence(position[i], position[j])
                delta0 = abs(length - self._tag_size)
                delta1 = abs(length - self._tag_size_angle)
                delta = min(delta0, delta1)
                delta_distence.append(delta)
                index.append((i, j))
        min_delta = min(delta_distence)
        if min_delta < qr_tolerance:
            index_min = delta_distence.index(min_delta)
        else:
            index_min = None
        point = index[index_min]

    def _calcu_distence(self, point0, point1):
        length = 0
        for i in range(len(point0)):
            length += pow(point0[i] - point1[i], 2)
        length = pow(length, 0.5)
        return length

    def _sort_points(self, points):
        points = points[points[:, 1].argsort()]
        if points[0][0] > points[1][0]:
            tmp_point = copy.deepcopy(points[0])
            points[0] = copy.deepcopy(points[1])
            points[1] = tmp_point
        if points[2][0] > points[3][0]:
            tmp_point = copy.deepcopy(points[2])
            points[2] = copy.deepcopy(points[3])
            points[3] = tmp_point
        return points


def main(cam):
    cam = cam
    pre_time = time.time()
    while not cv2.waitKey(1) == 27:
        frame, depth_data = cam.get_img()  # 获取一帧图像
        qr_datas, qr_four_conners = cam.qr_decode(frame)
        #print type(qr_four_conners[0])
        qr_pose = []
        if qr_datas:
            for i in range(len(qr_four_conners)):
                if qr_datas[i] == '00000000000':
                    position = cam.get_qrcode_pose(qr_four_conners[i], depth_data)
                    # print qr_pose[2]
                    # print position
        else:
            print('*******************')
        print time.time() - pre_time
        pre_time = time.time()
        # time.sleep(0.1)
        cv2.imshow("Video", frame)
        #cv2.waitKey(1)
    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    qr_code_pose_cam = qr_code_pose_cam()
    main(qr_code_pose_cam)
