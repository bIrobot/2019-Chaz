# Import the camera server
from cscore import CameraServer

# Import OpenCV and NumPy
import numpy as np

def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    usb1 = cs.startAutomaticCapture(dev=0)
    usb2 = cs.startAutomaticCapture(dev=1)

    usb1.setResolution(320, 240)
    usb2.setResolution(320, 240)

    cs.waitForever()
