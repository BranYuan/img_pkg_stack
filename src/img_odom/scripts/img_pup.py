# _*_ coding: UTF-8 _*_
import time
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

def main_loop():
    url = "rtsp://admin:@192.168.1.52/h265/main/av_stream"
    # cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(url)
    rospy.init_node('img_pup')
    img_pup = rospy.Publisher('img_raw', Image)
    img_bridge = cv_bridge.CvBridge()
    rate = 10
    rospy.Rate = rate
    while not rospy.is_shutdown():
        (ret, frame) = cap.read()  # 获取一帧图像
        frame = cv2.resize(frame, None, fx=0.4, fy=0.4)  # 改变图像大小
        img_msg = img_bridge.cv2_to_imgmsg(frame, 'bgr8')
        img_pup.publish(img_msg)
        cv2.imshow("Video", frame)
        cv2.waitKey(1)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main_loop()



