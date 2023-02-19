#!/home/snuzero/anaconda3/envs/tl/bin/python3

import rospy

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

import cv2
import numpy as np
import torch

from utils.augmentations import letterbox
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords, xyxy2xywh, apply_classifier
from utils.plots import colors, plot_one_box

from classifier.resnet import ResNet, BasicBlock
from classifier.utils import idx2str


WEIGHTS = "/home/snuzero/catkin_ws/src/zero_cheetah/slam/src/delivery_sign/delivery_sign_20210901_yolov5s_640.pt"
CLASSIFY = "/home/snuzero/catkin_ws/src/zero_cheetah/slam/src/delivery_sign/delivery_sign_20210907_resnet007.pt"
SIZE = 0.35  # Height in meters


class Smooth:
    def __init__(self):
        pass

    def process(self, out):
        delivery_dist = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0])
        m, t, tt = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], []
        for line, xyxy in out:
            x, y, w, h, c, p = line
            if c == 6 or w / h < 0.5 or h / w < 0.5:
                continue
            m[c] = max(m[c], h * p)
            t.append((line, xyxy))

        for line, xyxy in t:
            x, _, w, h, c, p = line
            if m[c] == h * p:
                d = (565 * SIZE) / (0.9 * 540 * h + 0.1 * 960 * w)
                #d /= np.cos(np.arctan((x - 0.5) * 960 / 565))
                delivery_dist[c] = d
                tt.append(xyxy + [c, p, d])

        return delivery_dist, tt


class Callback:
    def __init__(self):
        self.pub = rospy.Publisher("/delivery_dist", Float64MultiArray, queue_size=3)
        self.smooth = Smooth()

        # Initialize Model
        self.model = attempt_load(WEIGHTS, map_location="cuda:0")
        self.model.half()
        self.model(
            torch.zeros(1, 3, 960, 960)
            .to("cuda:0")
            .type_as(next(self.model.parameters()))
        )
        self.stride = int(self.model.stride.max())

        # Initialize Classifier
        self.modelc = ResNet(
            BasicBlock, [3, 3, 3], num_classes=7
        )  # uncomment if use ResNet20
        if isinstance(self.modelc, ResNet):
            self.modelc.to("cuda:0")
            self.modelc.load_state_dict(torch.load(CLASSIFY, map_location="cuda:0"))
        self.modelc.eval()
        self.names = [idx2str(n) for n in range(7)]

    def publisher(self, data):
        try:
            im0 = np.frombuffer(data.data, dtype=np.uint8).reshape(540, 960, -1)
            img = letterbox(im0, 960, stride=self.stride)[0]
            img = img.transpose((2, 0, 1))[::-1]
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to("cuda:0")
            img = img.half() / 255.0
            img = img[None]

            pred = self.model(img, augment=False, visualize=False)[0]
            pred = non_max_suppression(pred, 0.5, 0.5, max_det=1000)
            pred = apply_classifier(pred, self.modelc, img, im0)[0]
            pred[:, :4] = scale_coords(img.shape[2:], pred[:, :4], im0.shape).round()

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            out = []

            #Process Model Output
            for *xyxy, conf, cls in pred:
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                out.append((xywh + [int(cls.item()), conf.item()], xyxy))
            delivery_dist, tt = self.smooth.process(out)

            # Publish
            delivery_dist = Float64MultiArray(
                 data=np.array(delivery_dist).reshape(-1).tolist()
            )
            self.pub.publish(delivery_dist)

            # Visualizer
            for *xyxy, c, p, d in tt:
                 label = f"{self.names[c]} {p:.2f} {d:.2f}"
                 im0 = plot_one_box(
                     xyxy, im0, label=label, color=colors(c, True), line_width=2
                 )
            cv2.imshow("delivery_dist", im0)
            cv2.waitKey(1)
  
        except:
            cv2.destroyAllWindows()

    def subscriber(self):
        rospy.init_node("delivery_dist_node", anonymous=True)
        rospy.Subscriber("/camera_front", Image, self.publisher)
        rospy.spin()


if __name__ == "__main__":
    try:
        callback = Callback()
        callback.subscriber()
    except rospy.ROSInterruptException:
        print("Error!")


"""
CLASS ID
0 : A1
1 : A2
2 : A3
3 : B1
4 : B2
5 : B3
"""
