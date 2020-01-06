# _*_ coding: UTF-8 _*_
import time
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

def main_loop():
    url = "rtsp://admin:@192.168.52.52/h265/main/av_stream"
    cap = cv2.VideoCapture(0)
    print cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    print cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    half_width = int(width / 2)
    # cap = cv2.VideoCapture(url)
    rospy.init_node('img_pup')
    img_pup = rospy.Publisher('img_raw', Image)
    img_bridge = cv_bridge.CvBridge()
    rate = 5
    rospy.Rate = rate
    while not rospy.is_shutdown():
        (ret, frame0) = cap.read()  # 获取一帧图像
        frame_l = frame0[:, :half_width, :]
        frame_r = frame0[:, half_width:, :]
        frame = frame_l
        if ret:
            #frame = cv2.resize(frame, None, fx=0.5, fy=0.5)  # 改变图像大小
            img_msg = img_bridge.cv2_to_imgmsg(frame, 'bgr8')
            img_pup.publish(img_msg)
            cv2.imshow("Video", frame)
            cv2.waitKey(1)
        else:
            print 'err'
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main_loop()



