#!/usr/bin/env python

import cv2
import numpy as np
import socket
import struct
import math, time
from multiprocessing import Process

# ADDR = '192.168.1.51'
ADDR = '192.168.1.40'
WIDTH = 1280
HEIGHT = 720
FPS = 120
QUALITY = 30


class FrameSegment(object):
    """ 
    Object to break down image frame segment
    if the size of image exceed maximum datagram size 
    """
    MAX_DGRAM = 2**16
    MAX_IMAGE_DGRAM = MAX_DGRAM - 64 # extract 64 bytes in case UDP frame overflown
    def __init__(self, sock, port, addr=ADDR):
        self.s = sock
        self.port = port
        self.addr = addr

    def udp_frame(self, img1):
        """ 
        Compress image and Break down
        into data segments 
        """
        compress_img1 = cv2.imencode('.jpg', img1, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])[1]

        # fdsa
        dat = compress_img1.tostring() 
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
                             FPS])
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
        _, frame = cap.read()
        fs.udp_frame(frame)
    cap.release()
    cv2.destroyAllWindows()
    s.close()
    pass

def main():
    p1 = Process(target=run_sending, args=(0,))
    p2 = Process(target=run_sending, args=(2,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()