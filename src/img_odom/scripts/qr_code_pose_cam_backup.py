# _*_ coding: UTF-8 _*_

import time
import copy
import numpy
import pyzbar.pyzbar as pyzbar
import cv2
import rospy
import roslib
import tf

class qr_code_pose_cam:
    # 输入二维矩阵：
    # 第一维是要求解的二维码列表，第二维是对应二维码四个角点坐标
    # 求解并返回二维码位姿
    _tag_size = 199.0
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
        try:
            self._cap = cv2.VideoCapture(self._cam_id)
            self._cap.set(cv2.CAP_PROP_BRIGHTNESS, -40)
        except Exception as e:
            print e
            return False

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
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)     # 图像灰度化
            # # frame = cv2.medianBlur(frame, ksize=3)      # 中值滤波，去除噪声
            # frame = cv2.resize(frame, None, fx=0.5, fy=0.5)  # 改变图像大小
            return frame
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
            text = "{}".format(barcode_data[-4:-1] + barcode_data[-1])
            points = barcode.polygon
            points = numpy.array([[i.x, i.y] for i in points])
            cv2.polylines(image, [points], True, (0, 0, 255), 2)  # 画二维码框
            points = self._sort_points(points)
            first = points[0]
            second = points[1]
            third = points[2]
            fourth = points[3]
            a_length = pow(pow(first[0]-second[0], 2) + pow(first[1] - second[1], 2), 0.5)/21
            x1 = first[0] + (second[0] - first[0]) / 3
            y1 = first[1] + (second[1] - first[1]) / 3
            x2 = first[0] + (third[0] - first[0]) / 3
            y2 = first[1] + (third[1] - first[1]) / 3
            x3 = first[0] + (second[0] - first[0]) / 3 * 2
            y3 = first[1] + (second[1] - first[1]) / 3 * 2
            x4 = first[0] + (third[0] - first[0]) / 3 * 2
            y4 = first[1] + (third[1] - first[1]) / 3 * 2
            x5 = (x3 + x4) / 2
            y5 = (y3 + y4) / 2
            tmp_points = numpy.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4], [x5, y5]])
            # points = numpy.vstack((points, tmp_points))
            cv2.line(image, (x1, y1), (x5, y5), (0, 0, 255), 2)
            cv2.line(image, (x2, y2), (x5, y5), (0, 0, 255), 2)
            cv2.line(image, (x3, y3), (x4, y4), (0, 0, 255), 2)
            x = points[0][0]
            y = points[0][1]
            cv2.putText(image, text, (x, y + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 125), 2)
            print("[INFO] Found {} barcode: {} Time: {}".format(barcode_type, barcode_data, time.time()))
            datas.append(barcode_data)
            four_conners.append(points)
        return datas, four_conners

    def get_qrcode_pose(self, four_coner):
        # 求解二维码位姿
        four_coner = four_coner.astype(numpy.float64)
        ret_val, r_array, t_array = cv2.solvePnP(self._obj_points, four_coner, self._camera_matrix, self._dist_coeffs,
                                                 flags=cv2.SOLVEPNP_EPNP)
        axis_img_points, jac = cv2.projectPoints(self._axis, r_array, t_array, self._camera_matrix, self._dist_coeffs)
        qrcode_pose = [ret_val, r_array, t_array, axis_img_points, four_coner]
        #print r_array
        print t_array, r_array
        return qrcode_pose

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


def _test(cam):
    cam = cam
    pre_time = time.time()
    while not cv2.waitKey(1) == 27:
        frame = cam.get_img()  # 获取一帧图像
        qr_datas, qr_four_conners = cam.qr_decode(frame)
        #print type(qr_four_conners[0])
        qr_poses = []
        if qr_datas:
            for i in range(len(qr_four_conners)):
                qr_poses.append(cam.get_qrcode_pose(qr_four_conners[i]))
                if qr_datas[i] == '00000000004':
                    pass
        else:
            print('*******************')
        print time.time() - pre_time
        pre_time = time.time()
        #time.sleep(0.1)
        cv2.imshow("Video", frame)
        #cv2.waitKey(1)
    cam.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    qr_code_pose_cam = qr_code_pose_cam()
    _test(qr_code_pose_cam)