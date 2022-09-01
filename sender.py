#!/usr/bin/env python

import cv2
import numpy as np
import socket
import struct
import math, time
from multiprocessing import Process

from turbojpeg import TurboJPEG

ADDR = '192.168.1.51'
# ADDR = '192.168.1.40'
WIDTH = 1280
HEIGHT = 720
FPS = 120
QUALITY = 80

class BenchmarkTimer():
    def __init__(self, max_len = 60, prefix='') -> None:
        self.max_len = max_len
        self.dts = np.zeros(max_len)
        self.idx = 0
        self.prefix = prefix
        pass

    def save_time(self, dt):
        self.dts[self.idx] = dt
        self.idx += 1
        if self.idx == self.max_len:
            self.idx = 0
            self.print()
    def print(self):
        print(f'[{self.prefix}] dts (ms):', self.dts.mean() * 1000)

        pass

    def start(self):
        self.start_time = time.time()

    def check(self):
        dt = time.time() - self.start_time
        self.save_time(dt)
class FrameSegment(object):
    """ 
    Object to break down image frame segment
    if the size of image exceed maximum datagram size 
    """
    # MAX_DGRAM = 2**12
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64 # extract 64 bytes in case UDP frame overflown
    def __init__(self, sock, port, addr=ADDR):
        self.s = sock
        self.port = port
        self.addr = addr
        self.b = BenchmarkTimer(prefix=f'{port}')
        self.turbo_jpeg = TurboJPEG()

    def udp_frame(self, img1):
        """ 
        Compress image and Break down
        into data segments 
        """
        self.b.start()
        compress_img1 = self.turbo_jpeg.encode(img1,QUALITY)
        # compress_img1 = cv2.imencode('.jpg', img1, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])[1].tostring() 
        # compress_img1 = cv2.imencode('.jpg', img1, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])[1].tostring() 
        self.b.check()
        # fdsa
        dat = compress_img1
        size = len(dat)
        count = math.ceil(size/(self.MAX_IMAGE_DGRAM))
        array_pos_start = 0
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            self.s.sendto(struct.pack("B", count) +
                dat[array_pos_start:array_pos_end], 
                (self.addr, self.port)
                )
            array_pos_start = array_pos_end
            count -= 1


def run_sending(video_id = 0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = 41107 + video_id
    print (f'[{video_id}] open port with {port}')
    fs = FrameSegment(s, port)

    cap = cv2.VideoCapture(video_id, cv2.CAP_ANY, 
                            [cv2.CAP_PROP_FRAME_WIDTH, 
                             WIDTH, 
                             cv2.CAP_PROP_FRAME_HEIGHT, 
                             HEIGHT, 
                             cv2.CAP_PROP_FPS,
                             FPS, 
                             cv2.CAP_PROP_FOURCC,
                             cv2.VideoWriter_fourcc('U', 'Y', 'U', 'V')])
    befo_time = time.time()
    dts = np.zeros(60)
    idx = 0
    while (cap.isOpened()):
        dt = time.time() - befo_time
        dts[idx] = dt
        idx += 1

        if idx == 60:
            idx = 0
            print(f'[{video_id}] frame rate:', 1/dts.mean())

        befo_time = time.time()
        try:
            _, frame = cap.read()
        except:
            continue
        fs.udp_frame(frame)
    cap.release()
    cv2.destroyAllWindows()
    s.close()
    pass

def main():
    p1 = Process(target=run_sending, args=(2,))
    p2 = Process(target=run_sending, args=(0,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()