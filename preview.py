import numpy as np
import cv2
import serial
from control_lens import sendValue
import time
from Focus_match import cal_contrast

cam_id = 1
capture = cv2.VideoCapture(cam_id)
ser = serial.Serial('/dev/ttyUSB0')

lens_value = 70 # lens value shoud be int (1~254)
sendValue(ser, lens_value)
time.sleep(0.5) # shoud be more than 20ms

check = False
interval = 10

while(True):
    ret, frame = capture.read()
    # cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    cv2.imshow("frame", frame)
    if check is False:
        print(f'lens_value: {lens_value}, contrast_measure: {cal_contrast(frame)}')
        check = True

    arg = cv2.waitKey(1)

    if arg & 0xFF == ord("w"):
        if lens_value > interval:
            lens_value -= interval
            sendValue(ser, lens_value)
            time.sleep(0.5) # shoud be more than 20ms
            print(lens_value)
            print(np.shape(frame))
            check=True
        else:
            print('lens value is too low')

    if arg & 0xFF == ord("e"):
        if lens_value < 255-interval:
            lens_value += interval
            sendValue(ser, lens_value)
            time.sleep(0.5) # shoud be more than 20ms
            print(lens_value)
            print(cal_contrast(frame))
            check=True
        else:
            print('lens value is too high')
    
    if arg & 0xFF == ord("q"):
        break

capture.release()
ser.close()             # close port
cv2.destroyAllWindows()