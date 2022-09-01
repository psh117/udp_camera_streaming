#!/usr/bin/env python

import cv2
import numpy as np
import socket
import struct
import time
from multiprocessing import Process

ADDR = '192.168.1.40'
MAX_DGRAM = 2**16

def dump_buffer(s):
    """ Emptying buffer frame """
    while True:
        seg, addr = s.recvfrom(MAX_DGRAM)
        # print(seg[0])
        if struct.unpack("B", seg[0:1])[0] == 1:
            # print("finish emptying buffer")
            break

def run_viewer(video_id):
    port = 41107 + video_id
    print (f'[{video_id}] open port with {port}')
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ADDR, port))
    print (f'[{video_id}] success')
    dat = b''
    dump_buffer(s)
    befo_time = time.time()
    dts = np.zeros(60)
    idx = 0

    while True:
        dt = time.time() - befo_time
        dts[idx] = dt
        idx += 1

        if idx == 60:
            idx = 0
            print(f'[{video_id}] frame rate:', 1/dts.mean())

        befo_time = time.time()

        seg, addr = s.recvfrom(MAX_DGRAM)
        if struct.unpack("B", seg[0:1])[0] > 1:
            dat += seg[1:]
        else:
            dat += seg[1:]
            img = cv2.imdecode(np.fromstring(dat, dtype=np.uint8), 1)
            cv2.imshow('frame', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            dat = b''
            # dump_buffer(s)

    # cap.release()
    cv2.destroyAllWindows()
    s.close()


def main():
    p1 = Process(target=run_viewer, args=(0,))
    p2 = Process(target=run_viewer, args=(2,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()
