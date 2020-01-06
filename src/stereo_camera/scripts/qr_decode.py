# _*_ coding: UTF-8 _*_

import time
import copy
import numpy
import pyzbar.pyzbar as pyzbar
import cv2
from math import atan2

def qr_decode(image):
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
        points = sort_points(points)
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

def calcu_distence(point0, point1):
    length = 0
    for i in range(len(point0)):
        length += pow(point0[i] - point1[i], 2)
    length = pow(length, 0.5)
    return length

def sort_points(points):
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

