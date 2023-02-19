#!/home/snuzero/anaconda3/envs/tl/bin/python3

import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import os
import numpy as np
import cv2
from time import sleep, perf_counter
import copy

from camera import serial2id

FPS = 60
SERIAL = "EC6F8139"
CAM_MAT = np.array(
    [
        [755, 0.0, 640],
        [0.0, 755, 480],
        [0.0, 0.0, 1.0],
    ]
)  # Camera Matrix
DIS_MAT = np.array(
    [
        1.8381197097085436e-01,
        -4.1621081248642072e-01,
        1.4046580969321468e-04,
        -1.1921687101843280e-04,
        2.1254824941541825e-01,
    ]
)  # Distortion Matrix


def publisher():
    rospy.init_node("camera_front_node")
    pub = rospy.Publisher("/camera_front_for_cone", Image, queue_size=1)
    bridge = CvBridge()
    q = []

    # Initialize Webcam
    exp, exp_flag = 15, 12
    id = serial2id(SERIAL)
    cap = cv2.VideoCapture(id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_GAIN, 0)
    cap.set(cv2.CAP_PROP_BACKLIGHT, 0)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
    cap.set(cv2.CAP_PROP_CONTRAST, 144)
    cap.set(cv2.CAP_PROP_SATURATION, 176)
    cap.set(cv2.CAP_PROP_SHARPNESS, 128)
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, exp)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    cap.set(cv2.CAP_PROP_FOCUS, 0)
    cap.set(cv2.CAP_PROP_PAN, 0)
    cap.set(cv2.CAP_PROP_TILT, 0)
    cap.set(cv2.CAP_PROP_ZOOM, 100)
    os.system(f"v4l2-ctl -d {id} -c power_line_frequency=2")
    os.system(f"v4l2-ctl -d {id} -c led1_mode=1")
    os.system(f"v4l2-ctl -d {id} -c led1_frequency=0")
    os.system(f"v4l2-ctl -d {id} -c exposure_auto_priority=1")

    try:
        while not rospy.is_shutdown():
            t0 = perf_counter()
            ret, img = cap.read()

            # Process and Publish
            if ret:
                # Auto Exposure
                if exp_flag > 0:
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    gray = cv2.resize(gray, (320, 180), interpolation=cv2.INTER_AREA)
                    gray = np.mean(gray)
                    if 120 <= gray <= 140:
                        exp_flag = 0
                    elif gray < 120:
                        exp += 1
                        exp_flag -= 1
                    else:
                        exp -= 1
                        exp_flag -= 1
                    cap.set(cv2.CAP_PROP_EXPOSURE, exp)
                elif exp_flag == 0:
                    if exp >= 25:
                        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
                    exp_flag = -1

                # img = cv2.undistort(img, CAM_MAT, DIS_MAT)
                img = img[:, 160:-160]
                img = cv2.resize(img, (640, 480), cv2.INTER_AREA)
                q.append(copy.deepcopy(img))

                # Visualize
                cv2.imshow("camera_front", img)
                cv2.waitKey(1)

            # Wait
            while True:
                diff = t0 + 1 / FPS - perf_counter()
                if diff <= 0:
                    break
                else:
                    sleep(diff / 2)
 
            if len(q) > 1:
                camera_front = bridge.cv2_to_imgmsg(q.pop(0), "passthrough")
                pub.publish(camera_front)

    except:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        publisher()
    except rospy.ROSInterruptException:
        print("Error!")
