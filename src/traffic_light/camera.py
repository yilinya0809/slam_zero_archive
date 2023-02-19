import os
import subprocess

# 신호등 카메라
# SERIAL = "D07A1FEB"

# 표지판 카메라
# SERIAL = "EC6F8139"


def id2serial(id):
    p = subprocess.Popen(
        f"udevadm info --name=/dev/video{id} | grep ID_SERIAL=",
        stdout=subprocess.PIPE,
        shell=True,
    )
    out, _ = p.communicate()
    p.wait()
    return out.decode("utf-8").strip()[-8:]


def serial2id(serial):
    id = None
    for i in range(8):
        if serial == id2serial(i):
            id = i
            break
    return id
