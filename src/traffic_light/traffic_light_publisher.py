#!/home/snuzero/anaconda3/envs/tl/bin/python3

import rospy
from std_msgs.msg import Int32

import os
from time import sleep, perf_counter
import cv2
import numpy as np
import torch

from camera import serial2id
from utils.augmentations import letterbox
from models.experimental import attempt_load
from utils.plots import colors, plot_one_box
from utils.general import non_max_suppression, scale_coords, xyxy2xywh


class Smooth:
    def __init__(self):
        self.state = None
        self.mov_prob = np.array([0.2, 0.2, 0.2, 0.2, 0.2])

    def process(self, out):
        m, t, tt = 0.0, [], []
        for line, xyxy in out:
            x, y, w, h, c, p = line
            if not (0.05 < x < 0.95) or not (0.025 < y < 0.95):
                continue
            a = w * h * (p - 0.4)
            if a < 1e-5:
                continue
            m = max(m, a)
            t.append(line)
            tt.append(xyxy + [c, p])
        prob = np.zeros(5)
        if len(t) == 0:
            prob = np.array([0.05, 0.05, 0.05, 0.05, 0.8])
        else:
            for line in t:
                _, _, w, h, c, p = line
                prob[c] += (w * h * (p - 0.4) / m) ** 0.8
            prob /= np.sum(prob)
            prob *= 1 / (1 + np.exp((1e-5 - m) / 1e-3))
            prob += 1 / (1 + np.exp((m - 1e-5) / 1e-3)) / 5
        self.mov_prob = 0.875 * self.mov_prob + 0.125 * prob
        self.mov_prob /= np.sum(self.mov_prob)
        traffic_light = np.argmax(self.mov_prob)
        if traffic_light == 4:
            traffic_light = -1
        return traffic_light, tt


WEIGHTS = "/home/snuzero/catkin_ws/src/zero_cheetah/slam/src/traffic_light/traffic_light_20210824_yolov5m_640.pt"
FPS = 30


def publisher():
    rospy.init_node("traffic_light_node")
    pub = rospy.Publisher("/traffic_light", Int32, queue_size=3)

    # Initialize Model
    model = attempt_load(WEIGHTS, map_location="cuda:0")
    model.half()
    model(torch.zeros(1, 3, 640, 640).to("cuda:0").type_as(next(model.parameters())))
    stride = int(model.stride.max())
    names = model.module.names if hasattr(model, "module") else model.names

    # Initialize Webcam
    exp, exp_flag = 15, 12
    id = serial2id("D07A1FEB")
    cap = cv2.VideoCapture(id)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_GAIN, 0)
    cap.set(cv2.CAP_PROP_BACKLIGHT, 0)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
    cap.set(cv2.CAP_PROP_CONTRAST, 144)
    cap.set(cv2.CAP_PROP_SATURATION, 128)
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

    smooth = Smooth()
    t0 = perf_counter()

    try:
        while not rospy.is_shutdown():
            ret, im0 = cap.read()
            if ret:
                # Auto Exposure
                if exp_flag > 0:
                    gray = cv2.cvtColor(im0, cv2.COLOR_BGR2GRAY)
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

                im0 = im0[:, 240:-240]
                im0 = cv2.resize(im0, (640, 480), cv2.INTER_AREA)
                img = letterbox(im0, 640, stride=stride)[0]
                img = img.transpose((2, 0, 1))[::-1]
                img = np.ascontiguousarray(img)
                img = torch.from_numpy(img).to("cuda:0")
                img = img.half() / 255.0
                img = img[None]

                pred = model(img, augment=False, visualize=False)[0]
                pred = non_max_suppression(pred, 0.5, 0.5, max_det=1000)[0]
                pred[:, :4] = scale_coords(
                    img.shape[2:], pred[:, :4], im0.shape
                ).round()

                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
                out = []

                # Process Model Output
                for *xyxy, conf, cls in pred:
                    xywh = (
                        (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn)
                        .view(-1)
                        .tolist()
                    )
                    out.append((xywh + [int(cls.item()), conf.item()], xyxy))
                traffic_light, tt = smooth.process(out)

                pub.publish(traffic_light)

                # Visualize Result
                for *xyxy, c, p in tt:
                    label = f"{names[c]} {p:.2f}"
                    im0 = plot_one_box(
                        xyxy, im0, label=label, color=colors(c, True), line_width=2
                    )
                cv2.putText(
                    im0,
                    names[traffic_light] if traffic_light >= 0 else "X",
                    (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    3,
                    (0, 0, 0),
                    4,
                    cv2.LINE_AA,
                )
                cv2.imshow("traffic_light", im0)
                cv2.waitKey(1)

            # Wait
            while True:
                diff = t0 + 1 / FPS - perf_counter()
                if diff <= 0:
                    break
                else:
                    sleep(diff / 2)
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


"""
CLASS ID
-1 : No Traffic Light
 0 : Red
 1 : Red Left
 2 : Green
 3 : Green Left
"""
