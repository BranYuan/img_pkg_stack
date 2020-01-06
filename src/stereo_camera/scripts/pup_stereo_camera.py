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
    img_pup_r = rospy.Publisher('img_raw_r', Image, queue_size = 2)
    img_pup_l = rospy.Publisher('img_raw_l', Image, queue_size = 2)
    img_bridge = cv_bridge.CvBridge()
    rate = 5
    rospy.Rate = rate
    while not rospy.is_shutdown():
        (ret, frame0) = cap.read()  # 获取一帧图像
        frame_l = frame0[:, :half_width, :]
        frame_r = frame0[:, half_width:, :]
        if ret:
            img_msg_r = img_bridge.cv2_to_imgmsg(frame_r, 'bgr8')
            img_pup_r.publish(img_msg_r)
            img_msg_l = img_bridge.cv2_to_imgmsg(frame_l, 'bgr8')
            img_pup_l.publish(img_msg_l)
            cv2.imshow("Video", frame0)
            if cv2.waitKey(1) == ord('q'):
                break
        else:
            print 'err'
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main_loop()



