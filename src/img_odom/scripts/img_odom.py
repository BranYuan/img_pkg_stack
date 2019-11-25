# _*_ coding: UTF-8 _*_

import time
import copy
import numpy
import pyzbar.pyzbar as pyzbar
import cv2
import rospy
import roslib
import tf


def get_qrcode_poses(four_coners):
    # 输入二维矩阵：
    # 第一维是要求解的二维码列表，第二维是对应二维码四个角点坐标
    # 求解并返回二维码位姿
    qrcode_poses = []
    tag_size = 0.199
    tag_size_half = tag_size / 2
    # 相机内参
    '''
    # USB相机内参
    fx = 1367.482733
    fy = 1417.095603
    cx = 413.859812
    cy = 47.350579
    # 相机内参矩阵
    camera_matrix = numpy.array([[fx, 0, cx],
                                 [0, fy, cy],
                                 [0, 0, 1]], dtype=numpy.float64)
    # 相机畸变矩阵
    dist_coeffs = numpy.array([-0.231701, 0.023525, -0.039320, 0.001558, 0.0], dtype=numpy.float64)
    '''
    # 网络摄像头内参
    fx = 626.364407
    fy = 626.999916
    cx = 390.910434
    cy = 223.528294
    # 相机内参矩阵
    camera_matrix = numpy.array([[fx, 0, cx],
                                 [0, fy, cy],
                                 [0, 0, 1]], dtype=numpy.float64)
    # 相机畸变矩阵
    dist_coeffs = numpy.array([-0.463427, 0.213880, -0.005921, -0.001120, 0.000000], dtype=numpy.float64)
    # 二维码坐标轴长度矩阵
    axis = numpy.array([[tag_size_half, 0, 0],
                        [0, tag_size_half, 0],
                        [0, 0, tag_size_half]
                        ], dtype=numpy.float32)
    # 二维码四角点的3d坐标
    obj_points = numpy.array([[0, 0, 0],
                              [tag_size, 0, 0],
                              [0, -tag_size, 0],
                              [tag_size, -tag_size, 0]], dtype=numpy.float64)

    for points in four_coners:
        # 逐个求解二维码列表中各个二维码位姿
        points = points.astype(numpy.float64)
        ret_val, r_array, t_array = cv2.solvePnP(obj_points, points, camera_matrix, dist_coeffs,
                                                 flags=cv2.SOLVEPNP_ITERATIVE)
        axis_img_points, jac = cv2.projectPoints(axis, r_array, t_array, camera_matrix, dist_coeffs)
        qrcode_poses.append([ret_val, r_array, t_array, axis_img_points, points[0]])
        print(r_array, t_array)

    return qrcode_poses


def draw(image, corner, axis_img_points):
    corner = corner.astype(numpy.int)
    corner = tuple(corner)
    point1 = tuple(axis_img_points[0][0])
    point2 = tuple(axis_img_points[1][0])
    point3 = tuple(axis_img_points[2][0])
    image = cv2.line(image, corner, point1, (255, 0, 0), 5)
    image = cv2.line(image, corner, point2, (0, 255, 0), 5)
    image = cv2.line(image, corner, point3, (0, 0, 255), 5)
    return image


def sort_points(points):
    points_indexes = numpy.lexsort(points.T)  # last col (y axis) sort indexes
    if points[points_indexes[0]][0] > points[points_indexes[1]][0]:
        points_indexes[0], points_indexes[1] = points_indexes[1], points_indexes[
            0]  # row sort(x axis), two corner above of qrcode
    if points[points_indexes[2]][0] > points[points_indexes[3]][0]:
        points_indexes[2], points_indexes[3] = points_indexes[3], points_indexes[
            2]  # row sort(x axis), two corner below of qrcode
    points = numpy.array([points[i] for i in points_indexes])
    return points


def qr_decode(image):
    barcodes = pyzbar.decode(image)
    datas = []
    four_conners = []
    for barcode in barcodes:
        barcode_data = barcode.data.decode("utf-8")  # 解码data
        barcode_type = barcode.type
        text = "   {}: {}".format(barcode_type, barcode_data[-4:-1] + barcode_data[-1])

        points = barcode.polygon
        points = numpy.array([[i.x, i.y] for i in points])
        cv2.polylines(image, [points], True, (0, 0, 255), 2)  # 画圆
        points = sort_points(points)

        x = points[0][0]
        y = points[0][1]
        cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 125), 2)
        print("[INFO] Found {} barcode: {} Time: {}".format(barcode_type, barcode_data, time.time()))

        datas.append(barcode_data)
        four_conners.append(points)
    return image, datas, four_conners


def pup_tf(t_tuple, pitch, yaw, roll, frame, base_frame):
    if not rospy.is_shutdown():
        quaternion = tf.transformations.quaternion_from_euler(pitch, yaw, roll)
        broad_caster = tf.TransformBroadcaster()
        broad_caster.sendTransform(t_tuple, quaternion, rospy.Time.now(), frame, base_frame)
        return True
    else:
        return False


def main_loop():
    url = "rtsp://admin:@192.168.52.52/h265/main/av_stream"
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(url)
    rospy.init_node('img_odom_broadcaster')
    broad_caster = tf.TransformBroadcaster()
    broad_caster1 = tf.TransformBroadcaster()
    broad_caster2 = tf.TransformBroadcaster()
    while True:
        (ret, frame) = cap.read()  # 获取一帧图像
        # frame = cv2.flip(frame, 0)  # 翻转图像：水平翻转
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 图像灰度化
        frame = cv2.resize(frame, None, fx=0.4, fy=0.4)  # 改变图像大小
        # rows, cols = frame.shape  # 获取行列数
        # tf = cv2.getRotationMatrix2D((cols/2, rows/2), 180, 1) # 变换矩阵
        # frame = cv2.warpAffine(frame, tf, (cols, rows))  # 图像变换

        frame, qr_datas, qr_four_conners = qr_decode(frame)
        qr_poses = get_qrcode_poses(qr_four_conners)
        t_tuple = t_tuple1 = (0, 0, 0)
        quaternion = quaternion1 = (0, 0, 0, 1)
        qr_name = '00000000004'
        qr_name1 = '00000000003'
        for i in range(len(qr_poses)):
            frame = draw(frame, qr_poses[i][-1], qr_poses[i][-2])
            if i == 0:
                t_tuple = (qr_poses[i][2][0], qr_poses[i][2][1], qr_poses[i][2][2])
                quaternion = tf.transformations.quaternion_from_euler(qr_poses[i][1][0],
                                                                      qr_poses[i][1][1],
                                                                      qr_poses[i][1][2])
            elif i == 1:
                    t_tuple1 = (qr_poses[i][2][0], qr_poses[i][2][1], qr_poses[i][2][2])
                    quaternion1 = tf.transformations.quaternion_from_euler(qr_poses[i][1][0],
                                                                          qr_poses[i][1][1],
                                                                          qr_poses[i][1][2])
        broad_caster.sendTransform(t_tuple, quaternion, rospy.Time.now(), 'camera', qr_name)
        time.sleep(0.1)
        broad_caster1.sendTransform(t_tuple1, quaternion1, rospy.Time.now(), 'camera', qr_name1)
        time.sleep(0.1)
        broad_caster2.sendTransform((0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'camera1', 'camera')
        time.sleep(0.1)
        cv2.imshow("Video", frame)
        cv2.waitKey(1)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main_loop()