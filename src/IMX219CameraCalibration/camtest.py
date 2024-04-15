#!/usr/bin/env python2

# import cv2
# import gi
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst

# def read_cam():
#         cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
#         if cap.isOpened():
#                 cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
#                 while True:
#                         ret_val, img = cap.read();
#                         cv2.imshow('demo',img)
#                         if cv2.waitKey(30) == ord('q'): 
#                             break
#         else:
#                 print ("camera open failed")

#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#         print(cv2.getBuildInformation())
#         Gst.debug_set_active(True)
#         Gst.debug_set_default_threshold(3)
#         read_cam()





import sys
import cv2

def read_cam():
    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if cap.isOpened():
        cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
        while True:
            ret_val, img = cap.read()
            cv2.imshow('demo',img)
            cv2.waitKey(10)
    else:
        print("camera open failed")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    read_cam()









# import numpy as np
# import cv2
# # import time 
# from gst_cam import camera

# # print(cv2.__version__)
# # w, h = 1280, 720
# # cap = cv2.VideoCapture(camera(0, w, h))
# cap = cv2.VideoCapture(0)
# assert cap.isOpened()
# cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

# # time.sleep(1)
# while(True):
#     # Capture frame-by-frame
#     ret, frame = cap.read()

#     # Our operations on the frame come here
#     # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     # Display the resulting frame
#     cv2.imshow('frame',frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # When everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()







# import cv2
# import time
# try:
#     from  Queue import  Queue
# except ModuleNotFoundError:
#     from  queue import  Queue

# import  threading
# import signal
# import sys


# # def signal_handler(sig, frame):
# #     print('You pressed Ctrl+C!')
# #     sys.exit(0)
# # signal.signal(signal.SIGINT, signal_handler)


# def gstreamer_pipeline(
#     capture_width=1280,
#     capture_height=720,
#     display_width=640,
#     display_height=360,
#     framerate=60,
#     flip_method=0,
# ):
#     return (
#         "nvarguscamerasrc ! "
#         "video/x-raw(memory:NVMM), "
#         "width=(int)%d, height=(int)%d, "
#         "format=(string)NV12, framerate=(fraction)%d/1 ! "
#         "nvvidconv flip-method=%d ! "
#         "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
#         "videoconvert ! "
#         "video/x-raw, format=(string)BGR ! appsink"
#         % (
#             capture_width,
#             capture_height,
#             framerate,
#             flip_method,
#             display_width,
#             display_height,
#         )
#     )

# class FrameReader(threading.Thread):
#     queues = []
#     _running = True
#     camera = None
#     def __init__(self, camera, name):
#         threading.Thread.__init__(self)
#         self.name = name
#         self.camera = camera
 
#     def run(self):
#         while self._running:
#             _, frame = self.camera.read()
#             while self.queues:
#                 queue = self.queues.pop()
#                 queue.put(frame)
    
#     def addQueue(self, queue):
#         self.queues.append(queue)

#     def getFrame(self, timeout = None):
#         queue = Queue(1)
#         self.addQueue(queue)
#         return queue.get(timeout = timeout)

#     def stop(self):
#         self._running = False

# class Previewer(threading.Thread):
#     window_name = "Arducam"
#     _running = True
#     camera = None
#     def __init__(self, camera, name):
#         threading.Thread.__init__(self)
#         self.name = name
#         self.camera = camera
    
#     def run(self):
#         self._running = True
#         while self._running:
#             cv2.imshow(self.window_name, self.camera.getFrame(2000))
#             keyCode = cv2.waitKey(16) & 0xFF
#         cv2.destroyWindow(self.window_name)

#     def start_preview(self):
#         self.start()
#     def stop_preview(self):
#         self._running = False

# class Camera(object):
#     frame_reader = None
#     cap = None
#     previewer = None

#     def __init__(self):
#         self.open_camera()

#     def open_camera(self):
#         self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
#         if not self.cap.isOpened():
#             raise RuntimeError("Failed to open camera!")
#         if self.frame_reader == None:
#             self.frame_reader = FrameReader(self.cap, "")
#             self.frame_reader.daemon = True
#             self.frame_reader.start()
#         self.previewer = Previewer(self.frame_reader, "")

#     def getFrame(self):
#         return self.frame_reader.getFrame()

#     def start_preview(self):
#         self.previewer.daemon = True
#         self.previewer.start_preview()

#     def stop_preview(self):
#         self.previewer.stop_preview()
#         self.previewer.join()
    
#     def close(self):
#         self.frame_reader.stop()
#         self.cap.release()

# if __name__ == "__main__":
#     camera = Camera()
#     camera.start_preview()
#     time.sleep(10)
#     camera.stop_preview()
#     camera.close()