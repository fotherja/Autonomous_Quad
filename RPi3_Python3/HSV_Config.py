import tkinter as tk
from threading import Thread,Event
from multiprocessing import Array
from ctypes import c_int32
import cv2
import numpy as np


class CaptureController(tk.Frame):
    NSLIDERS = 6
    def __init__(self,parent):
        tk.Frame.__init__(self)
        self.parent = parent

        # create a synchronised array that other threads will read from
        self.ar = Array(c_int32,self.NSLIDERS)

        # create NSLIDERS Scale widgets
        self.sliders = []
        for ii in range(self.NSLIDERS):
            # through the command parameter we ensure that the widget updates the sync'd array
            s = tk.Scale(self, from_=0, to=255, orient=tk.HORIZONTAL,
                         command=lambda pos,ii=ii:self.update_slider(ii,pos))
            s.pack()
            self.sliders.append(s)

        # Define a quit button and quit event to help gracefully shut down threads
        tk.Button(self,text="Quit",command=self.quit).pack()
        self._quit = Event()
        self.capture_thread = None

    # This function is called when each Scale widget is moved
    def update_slider(self,idx,pos):
        self.ar[idx] = c_int32(int(pos))

    # This function launches a thread to do video capture
    def start_capture(self):
        self._quit.clear()
        # Create and launch a thread that will run the video_capture function
        self.capture_thread = Thread(target=video_capture, args=(self.ar,self._quit))
        self.capture_thread.daemon = True
        self.capture_thread.start()

    def quit(self):
        self._quit.set()
        try:
            self.capture_thread.join()
        except TypeError:
            pass
        self.parent.destroy()


# This function simply loops over and over, printing the contents of the array to screen
def video_capture(ar,quit):
    cap = cv2.VideoCapture(0)

    while(1):
        while(1):
            ret, frame = cap.read()
            if(ret):
                break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([ar[0],ar[1],ar[2]])
        upper_blue = np.array([ar[3],ar[4],ar[5]])
        #lower_blue = np.array([154,128,45])
        #upper_blue = np.array([218,255,255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        res = cv2.bitwise_and(frame,frame, mask= mask)

        #cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)
        cv2.imshow('res',res)

        cv2.waitKey(5)

if __name__ == "__main__":
    root = tk.Tk()
    selectors = CaptureController(root)
    selectors.pack()
    selectors.start_capture()
    root.mainloop()












    
