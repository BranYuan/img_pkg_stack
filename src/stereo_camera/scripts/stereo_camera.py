# _*_ coding: UTF-8 _*_
import time
import datetime
import numpy as np
import cv2
import config
from qr_decode import *

def callback_func(e, x, y, f, p):
    if e == cv2.EVENT_LBUTTONDOWN:
        print x, y 
        print threeD[y][x]

def main_loop():
    pre_time = time.time()
    cv2.namedWindow('Video')
    cv2.namedWindow('depth')
    cv2.createTrackbar('num', 'depth', 0, 10, lambda x: None)
    cv2.createTrackbar('blockSize', 'depth', 5, 255, lambda x: None)
    cap = cv2.VideoCapture(0)
    cv2.setMouseCallback('depth', callback_func, None)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 20)
    cap.set(cv2.CAP_PROP_CONTRAST, 0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print cap.get(cv2.CAP_PROP_BUFFERSIZE)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    half_width = int(width / 2)
    while True:
        (ret, frame) = cap.read()  # 获取一帧图像
        if not ret:
            print 'err'
            break
        frame_l = frame[:, :half_width, :]
        frame_r = frame[:, half_width:, :]
        # 图像重构
        img_rectified_l = cv2.remap(frame_l, config.left_map1, config.left_map2, cv2.INTER_LINEAR)
        img_rectified_r = cv2.remap(frame_r, config.right_map1, config.right_map2, cv2.INTER_LINEAR)
        # 获取灰度图，为stereoBM做准备
        # img_l = cv2.cvtColor(img_rectified_l, cv2.COLOR_BGR2GRAY)
        # img_r = cv2.cvtColor(img_rectified_r, cv2.COLOR_BGR2GRAY)
        img_l = img_rectified_l
        img_r = img_rectified_r

        # 两个trackbar用来调节不同的参数查看效果
        num = cv2.getTrackbarPos("num", "depth")
        blockSize = cv2.getTrackbarPos("blockSize", "depth")
        if blockSize % 2 == 0:
            blockSize += 1
        if blockSize < 5:
            blockSize = 5
        if img_l.ndim == 2:
            img_channels = 1
        else:
            img_channels = 3
        # 根据Block Maching方法生成差异图（opencv里也提供了SGBM/Semi-Global Block Matching算法，有兴趣可以试试）
        param = {'minDisparity': 0,
             'numDisparities': 128,
             'blockSize': blockSize,
             'P1': 8 * img_channels * blockSize ** 2,
             'P2': 32 * img_channels * blockSize ** 2,
             'disp12MaxDiff': 1,
             'preFilterCap': 63,
             'uniquenessRatio': 15,
             'speckleWindowSize': 400,
             'speckleRange': 2,
             'mode': cv2.STEREO_SGBM_MODE_SGBM_3WAY
             }
        #stereo = cv2.StereoBM_create(numDisparities=16*num, blockSize=blockSize)
        stereo = cv2.StereoSGBM_create(**param)
        disparity = stereo.compute(img_l, img_r)
        disp = cv2.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # 将图片扩展至3d空间中，其z方向的值则为当前的距离
        global threeD
        threeD = cv2.reprojectImageTo3D(disparity.astype(np.float32)/16., config.Q)
        data, conners = qr_decode(img_l)
        if conners:
            #conners = sort_points(conners[0])
            conner = conners[0]
            p1_x, p1_y = conner[0][0], conner[0][1]
            p2_x, p2_y = conner[1][0], conner[1][1]
            p1 = threeD[p1_y][p1_x] * (199.0/140)
            p2 = threeD[p2_y][p2_x] * (199.0/140)
            length = int(calcu_distence(p1, p2) * 1000)
            if not length > 500:
                print length
        img_l = cv2.resize(img_l, None, fx = 0.4, fy = 0.4)
        # print datetime.datetime.now()
        disp = cv2.resize(disp, None, fx = 0.4, fy = 0.4)
        #cv2.imshow("Video", img_l)
        #cv2.imshow("depth", disp)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        print time.time() - pre_time
        pre_time = time.time()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main_loop()



