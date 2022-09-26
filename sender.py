#!/usr/bin/env python

from glob import glob
import cv2
import yaml
import numpy as np
import socket
import struct
import math, time
from multiprocessing import Process
from dataclasses import dataclass

from turbojpeg import TurboJPEG

# ADDR = '192.168.1.51'
# ADDR = '127.0.0.1'
# ADDR = '192.168.1.40'
ADDR = '192.168.0.33'
WIDTH = 1920
HEIGHT = 1200
FPS = 60
QUALITY = 80
BASE_PORT = 41107

shutdown = False

@dataclass
class args_left:
    yamlfile = 'avmL_1A30920B.yaml'


@dataclass
class args_right:
    yamlfile = 'avmR_330B8306.yaml'

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
        # print(count)

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
            
        # if count > 1:
        #     print('2')
        #     return
        # while count:
        #     array_pos_end = min(size, array_pos_start + self.MAX_IMAGE_DGRAM)
        #     self.s.sendto(struct.pack("B", count) +
        #         dat[array_pos_start:array_pos_end], 
        #         (self.addr, self.port)
        #         )
        #     array_pos_start = array_pos_end
        #     count -= 1


def run_sending(video_id = 0, args=[]):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    port = BASE_PORT + video_id
    print (f'[{video_id}] open port with {port}')
    fs = FrameSegment(s, port)

    codec = cv2.VideoWriter_fourcc('M','J','P','G') 
    # codec = cv2.VideoWriter_fourcc('U','Y','V','Y')
    cap = cv2.VideoCapture(video_id, cv2.CAP_ANY, [cv2.CAP_PROP_FRAME_WIDTH, 
                              WIDTH, 
                              cv2.CAP_PROP_FRAME_HEIGHT, 
                              HEIGHT, 
                              cv2.CAP_PROP_FPS,
                              FPS,
                              cv2.CAP_PROP_FOURCC,
                              codec ])
                              
    with  open(args.yamlfile) as f:
        a = yaml.full_load(f)
        
    dts = np.zeros(60)
    idx = 0
    
    mtx = np.array(a['mtx'])
    dist = np.array(a['dist'])
    roi = np.array(a['roi'])
    pts1 = np.array(a['pts1'], dtype=np.float32)
    newcameramtx = np.array(a['newcameramtx'])

    
    # width, height = 720,720
    # # after installation, modify here
    # pts1 = np.float32([[365,0],[740,0],[0,719],[1279,719
    #                                             ]])
    # pts2 = np.float32([[0,0],[width,0],[0,height],[width,height]])
    
    # 
    width = 500
    height = 400
    pts2 = np.float32([[0, 0], [width - 1, 0],[width - 1, height - 1], [0, height - 1]])

    mtrx = cv2.getPerspectiveTransform(pts1, pts2)

    befo_time = time.time()
    while (cap.isOpened() and not shutdown):
        dt = time.time() - befo_time
        dts[idx] = dt
        idx += 1

        if idx == 60:
            idx = 0
            print(f'[{video_id}] frame rate:', 1/dts.mean())
            print(frame.shape)

        befo_time = time.time()
        try:
            _, frame = cap.read()

            h, w = frame.shape[:2]
            mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
            dst = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]

            # 원근 변환 적용
            result = cv2.warpPerspective(dst, mtrx, (width, height))
            # cv2.imwrite(f'out_{port}.jpg', result)
            
            # matrix = cv2.getPerspectiveTransform(pts1, pts2)
            # frame = cv2.warpPerspective(frame, matrix, (width,height))
            
            fs.udp_frame(result)
        except:
            continue
    cap.release()
    cv2.destroyAllWindows()
    s.close()
    pass

def main():
    global shutdown
    p1 = Process(target=run_sending, args=(0,args_left))
    p2 = Process(target=run_sending, args=(2,args_right ))
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
