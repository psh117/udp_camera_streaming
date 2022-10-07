#!/usr/bin/env python

import numpy as np
import socket
import struct
import math, time
from multiprocessing import Process
import subprocess

from v4l2py import Device
from turbojpeg import TurboJPEG
import time

from nvjpeg import NvJpeg

#ADDR = '192.168.0.14'
ADDR = '192.168.0.33'
# ADDR = '192.168.1.40'
# WIDTH = 1280
# HEIGHT = 720
# FPS = 120
WIDTH = 1920
HEIGHT = 1200
FPS = 114
QUALITY = 40

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
        self.a = BenchmarkTimer(prefix='1')
        self.b = BenchmarkTimer(prefix='2')
        self.c = BenchmarkTimer(prefix='3')
        self.turbo_jpeg = TurboJPEG()

        self.first = True
        # self.compress_img1
        self.nj = NvJpeg()

    def udp_frame(self, img1):
        """ 
        Compress image and Break down
        into data segments 
        """
        try:
            # com1 = 
            # if not self.first:
            self.a.start()
            # compress_img1 = self.turbo_jpeg.scale_with_quality(img1,quality=QUALITY)
            com1 = self.nj.decode(img1)
            #com1 = self.turbo_jpeg.decode(img1)
            self.a.check()

            self.b.start()
            compress_img1 = self.nj.encode(com1,40)
            #compress_img1 = self.turbo_jpeg.encode(com1,40)
            self.b.check()

            # self.c.start()
            # temp= self.nj.decode(compress_img1)

            # # temp=self.turbo_jpeg.decode(compress_img1)
            # #compress_img1 = self.turbo_jpeg.encode(com1,40)
            # self.c.check()
                # self.first = false

            self.first = False

            # self.a.start()
            # com1 = self.turbo_jpeg.decode(img1)
            # self.a.check()

            # self.b.start()
            # compress_img1 = self.turbo_jpeg.encode(com1, QUALITY)
            # self.b.check()

            # cv2.imdecode 
            # cv2.imencode

        except:
            print('error')
            return
        #compress_img1 = cv2.imencode('.jpg', img1, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])[1].tostring() 
        # compress_img1 = cv2.imencode('.jpg', img1, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])[1].tostring() 

        # fdsa
        dat = compress_img1
        size = len(dat)
        count = math.ceil(size/(self.MAX_IMAGE_DGRAM))
        array_pos_start = 0
        # if count > 2:
        #     print('shape', img1.shape, 'count', count)
            # return
        start = True
        while count:
            array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
            if start:
                self.s.sendto(struct.pack("B", 237) + struct.pack("B", count) +
                    dat[array_pos_start:array_pos_end], 
                    (self.addr, self.port)
                    )
                start = False
            else:
                self.s.sendto(struct.pack("B", 210) + struct.pack("B", count) +
                    dat[array_pos_start:array_pos_end], 
                    (self.addr, self.port)
                    )

            array_pos_start = array_pos_end
            count -= 1

def run_sending(cam_serial = None):
    """
    41107 : left -- 1E328E0B
    41109 : right -- 322F8B0B
    
    /bin/udevadm info --name=/dev/video2 | grep SERIAL_SHORT | cut -d '=' -f2
    """
    cam_serial_to_port = {'1E328E0B':41107,'322F8B0B':41109}
    cam_serial_to_name = {'1E328E0B':'LEFT ','322F8B0B':'RIGHT'}

    find_serial = False

    for i in range(50):
        try:
            cam_test = Device.from_id(i)
            if cam_test.info.card == 'See3CAM_24CUG' and cam_test.info.capabilities == 69206017:
                print(f'Found See3CAM_24CUG at /dev/video{i}')

                find_serial_cmd = f"/bin/udevadm info --name=/dev/video{i} | grep SERIAL_SHORT | cut -d '=' -f2"
                serial_out = result = subprocess.check_output(find_serial_cmd, shell=True, text=True)[:-1]
                if serial_out == cam_serial:
                    port = cam_serial_to_port[serial_out]
                    print(f'Found that video{i} -- {cam_serial}')
                    cam = cam_test
                    find_serial = True
                    break
        except:
            pass

    if find_serial is False:
        print('no matching camera serial. check the codes.')
        return

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print (f'[{port}] open port for {cam_serial_to_name[cam_serial]}')
    fs = FrameSegment(s, port)

    cam.video_capture.set_format(WIDTH, HEIGHT, 'MJPG')
    befo_time = time.time()
    dts = np.zeros(60)
    idx = 0
    
    for i, frame in enumerate(cam):
        dt = time.time() - befo_time
        dts[idx] = dt
        idx += 1

        if idx == 60:   
            idx = 0
            print(f'[{port}] frame rate:', 1/dts.mean())

        befo_time = time.time()
        fs.udp_frame(frame)
    s.close()
    pass

def main():
    p1 = Process(target=run_sending, args=('1E328E0B',))
    p2 = Process(target=run_sending, args=('322F8B0B',)) 
    p1.start()
    p2.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('gracefully shutdown ...')
        shutdown = True
        p1.terminate()
        p2.terminate()
        
    p1.join()
    p2.join()

if __name__ == "__main__":
    main()
