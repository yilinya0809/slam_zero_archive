#!/home/snuzero/anaconda3/envs/tl/bin/python3

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from time import perf_counter, sleep

import cv2
import numpy as np
import torch

from utils.augmentations import letterbox
from models.experimental import attempt_load
from utils.plots import colors, plot_one_box
from utils.general import non_max_suppression, scale_coords, xyxy2xywh


WEIGHTS = "/home/snuzero/catkin_ws/src/zero_cheetah/slam/src/traffic_light/traffic_cone_20210828_yolov5s_640.pt"
MASK = (
    np.array(
        [
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 1, 1, 1, 0, 0],
            [0, 1, 1, 1, 1, 1, 0],
            [1, 1, 1, 1, 1, 1, 1],
        ],
        dtype=np.uint8,
    )
    * 255
)


class Callback:
    def __init__(self):
        self.pub = rospy.Publisher("/cone_box", Float32MultiArray, queue_size=2)
        self.model = attempt_load(WEIGHTS, map_location="cuda:0")
        self.model.half()
        self.model(
            torch.zeros(1, 3, 640, 640)
            .to("cuda:0")
            .type_as(next(self.model.parameters()))
        )
        self.stride = int(self.model.stride.max())

    def publisher(self, data):
        try:
            t0 = perf_counter()
            im0 = np.frombuffer(data.data, dtype=np.uint8).reshape(480, 640, -1)
            im0_blur = cv2.resize(im0, dsize=(320, 240), interpolation=cv2.INTER_AREA)
            im0_blur = cv2.GaussianBlur(im0_blur, (5, 5), 0)
            im0_blur = cv2.cvtColor(im0_blur, cv2.COLOR_BGR2HSV)
            im0_blur = im0_blur[:, :, 0].copy()
            im0_blur = np.where(im0_blur < 60, 1, 0) + np.where(
                (80 < im0_blur) & (im0_blur < 140), -1, 0
            )

            img = letterbox(im0, 640, stride=self.stride)[0]
            img = img.transpose((2, 0, 1))[::-1]
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to("cuda:0")
            img = img.half() / 255.0
            img = img[None]

            pred = self.model(img, augment=False, visualize=False)[0]
            pred = non_max_suppression(pred, 0.5, 0.5, max_det=1000)[0]
            pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], im0.shape).round()

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            out, tt = [], []

            for *xyxy, _, p in pred:
                x, y, w, h = (
                    (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                )
                blue, yellow = 0, 0
                height, width = im0_blur.shape
                v_min = round(height * (y - 0.45 * h))
                v_max = round(height * (y + 0.45 * h))
                h_min = round(width * (x - 0.5 * w))
                h_max = round(width * (x + 0.5 * w))
                crop = im0_blur[v_min:v_max, h_min:h_max].copy().astype(np.float32)
                mask = cv2.resize(
                    MASK, dsize=(crop.shape[1], crop.shape[0]), interpolation=cv2.INTER_LINEAR
                ).astype(np.float32)

                if np.sum(crop * mask) > 0:
                    color = 1.0
                else:
                    color = 0.0

                out.append([x, y, w, h, color])
                tt.append(xyxy + [color, p])

            cone_box = Float32MultiArray(data=np.array(out).reshape(-1).tolist())
            self.pub.publish(cone_box)

            # Visulaize
            for *xyxy, c, p in tt:
                im0 = plot_one_box(
                    xyxy, im0, label=f"{p:.2f}", color=colors(int(c), True), line_width=2
                )
            cv2.imshow("cone_box", im0)
            cv2.waitKey(1)

        except e:
            cv2.destroyAllWindows()

    def subscriber(self):
        rospy.init_node("cone_box_node")
        rospy.Subscriber("/camera_front_for_cone", Image, self.publisher)
        rospy.spin()


if __name__ == "__main__":
    try:
        callback = Callback()
        callback.subscriber()
    except rospy.ROSInterruptException:
        print("Error!")
