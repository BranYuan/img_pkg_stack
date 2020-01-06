#_*_coding:utf-8_*_
# usr/bin/python
import sys
import numpy as np
import cv2
from openni import openni2
from openni import _openni2 as c_api

def main():
    cap = cv2.VideoCapture(0)
    (ret, color_frame) = cap.read()
    openni2.initialize()
    device = openni2.Device.open_any()
    device.set_depth_color_sync_enabled(True)
    depth_stream = device.create_depth_stream()
    depth_stream.start()
    while True:
        print("#################")
        frame = depth_stream.read_frame()
        ret, color_frame = cap.read()
        frame_data = frame.get_buffer_as_uint8()
        position = openni2.convert_depth_to_world(depth_stream, 100, 100, frame_data[10000])
        print position
        cv2.imshow("image", color_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
    depth_stream.stop()
    openni2.unload()

if __name__ == "__main__":
    main()
