# _*_ coding: UTF-8 _*_
import numpy as np
import cv2

def undistortion(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    print ('roi', roi)
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + h]
    return dst


def calibrate(cap):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    Nx_cor = 9
    Ny_cor = 6
    width = 21.875      # 棋盘格边长，单位m

    objp = np.zeros((Nx_cor * Ny_cor, 3), np.float32)
    objp[:, :2] = np.mgrid[0:Nx_cor, 0:Ny_cor].T.reshape(-1, 2)
    objp = objp * width    # 3D point value in real world
    objpoints = []      # 3d points in real world space
    imgpoints = []      # 2d points in image plane
    count = 0       # count 用来标志成功检测到的棋盘格画面数量
    while True:
        ret, frame = cap.read()     # 获取一帧图像
        frame = cv2.resize(frame, None, fx=0.5, fy=0.5)  # 改变图像大小
        if cv2.waitKey(1) & 0xFF == ord(' '):
            # 空格按下，获取一帧图像
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (Nx_cor, Ny_cor), None)
            print('get one img %s', count)
            # 如果发现棋盘格，则讲objpoints 和 imgpoints 加入list
            if ret == True:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
                objpoints.append(objp)
                imgpoints.append(corners)
                cv2.drawChessboardCorners(frame, (Nx_cor, Ny_cor), corners, ret)
                count += 1
                if count > 20:
                    break
        # 显示图像结果
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    global mtx, dist
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(mtx, dist)
    mean_error = 0
    for i in xrange(len(objpoints)):
        imgpoints2, jac = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
        print 'total error: ', mean_error / len(objpoints)
        np.savez('calibrate.npz', mtx=mtx, dist=dist[0:4])
    '''
    (array([[735.46080545,   0.        , 487.04677637],
       [  0.        , 732.3286531 , 259.41027867],
       [  0.        ,   0.        ,   1.        ]]), 
       
       array([[-4.54350100e-01,  2.83077330e-01, -2.74053940e-04,
         7.77758472e-04, -1.10242838e-01]]))
    # mtx = [[,599.60234739 0, ]380.40832195
             [0, 599.22310639, 211.32155849]
             [0,0,1]]
    # dist = [[-0.446054, 0.311579, -0.396020]]
    '''
'''
def undistortion(img, mtx, dist):
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    if roi != (0, 0, 0, 0):
        dst = dst[y:y + h, x:x + h]
    return dst
'''

if __name__ == '__main__':
    url = "rtsp://admin:@192.168.52.52/h265/main/av_stream"
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(url)
    # calibrate(cap)
    mtx = []
    dist = []
    try:
        npzfile = np.load('calibrate.npz')
        print npzfile
        mtx = npzfile['mtx']
        dist = npzfile['dist']
    except IOError:
        calibrate(cap)
    print('dist', dist[0:4])
    while True:
        ret, frame = cap.read()
        frame = undistortion(frame, mtx, dist[0:4])
        frame = cv2.resize(frame, None, fx=0.5, fy=0.5)  # 改变图像大小
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()