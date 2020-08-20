import numpy as np
import argparse
import cv2
import time
import serial
from focus import cal_contrast, make_fibonacci
from control_lens import sendValue


class AutofocusCamera:
    def __init__(self, cam_id, port_name, wait_time=0.5):
        self.capture = cv2.VideoCapture(cam_id)
        self.ser = serial.Serial(port_name)
        self.wait_time = wait_time

    def focus_cam(
        self, N, min, max, tolerance, ROI=None, gray_scale=True, scale=1, min_interval=5
    ):
        fibonacci_list = make_fibonacci(N)

        x, y = min, max  # lens value shoud be int (1~254)
        sendValue(self.ser, x)
        time.sleep(self.wait_time)
        _, img_x = self.capture.read()
        sendValue(self.ser, y)
        time.sleep(self.wait_time)
        _, img_y = self.capture.read()
        if not gray_scale:
            img_x = cv2.cvtColor(img_x, cv2.COLOR_BGR2GRAY)
            img_y = cv2.cvtColor(img_y, cv2.COLOR_BGR2GRAY)
        fx = cal_contrast(img_x, ROI)
        fy = cal_contrast(img_y, ROI)
        error = abs(fx - fy)

        for i in range(N - 2):
            L = y - x
            if error <= tolerance:
                i = i - 1
                break

            print("-" * 50)
            print(f"\niteration: {i} ")
            print(f"[x, y]: [{x}, {y}]")
            print(f"error: {error}")

            interval = (
                fibonacci_list[N - i - 3] / (fibonacci_list[N - i - 1] * scale)
            ) * L

            if interval < min_interval:
                print("iterval is too small")
                print("end iteration")
                break

            x1 = int(x + interval)
            y1 = int(y - interval)
            sendValue(self.ser, x1)
            time.sleep(self.wait_time)
            _, img_x1 = self.capture.read()
            sendValue(self.ser, y1)
            time.sleep(self.wait_time)
            _, img_y1 = self.capture.read()
            if not gray_scale:
                img_x1 = cv2.cvtColor(img_x, cv2.COLOR_BGR2GRAY)
                img_y1 = cv2.cvtColor(img_y, cv2.COLOR_BGR2GRAY)
            fx1 = cal_contrast(img_x1, ROI)
            fy1 = cal_contrast(img_y1, ROI)
            print(f"x1: {x1}, fx1:{fx1}")
            print(f"y1: {y1}, fy1:{fy1}")
            if fx1 < fy1:
                print(f"x is changed {x} -> {x1}\n")
                x = x1
                fx = fx1
            else:
                print(f"y is changed {y} -> {y1}\n")
                y = y1
                fy = fy1

            error = abs(fx - fy)

        print("Search is finished!")
        print("-" * 10)
        print(f"total iteration: {i+1} ")
        print(f"[x, y]: [{x}, {y}]")
        print(f"final error: {error}\n\n")
        if fx > fy:
            print(f"maximum value: {fx} ,index: {x}")
            return x, fx
        else:
            print(f"maximum value: {fy} ,index: {y}")
            return y, fy


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Auto_focus algorithm for a camera")

    parser.add_argument("--cam_id", required="True", help="the cam id number")
    parser.add_argument("--port", required="True", help="Liquid lens port name")
    parser.add_argument(
        "--max_iter", type=int, default=20, help="max iteration in search"
    )
    parser.add_argument(
        "--min", type=int, default=1, help="minimum value in the search area"
    )
    parser.add_argument(
        "--max", type=int, default=254, help="maximum value in the search area"
    )
    parser.add_argument("--tol", type=int, default=0, help="tolerance value in search")
    parser.add_argument(
        "--gray",
        type=bool,
        default=True,
        help="wheather the camera image is gray scale or not",
    )
    parser.add_argument(
        "--wait", type=float, default=0.5, help="waiting time in serial communication"
    )
    parser.add_argument("--scale", type=int, default=1, help="a scale factor in search")
    parser.add_argument(
        "--mit", type=int, default=5, help="the minimum interval in search"
    )
    parser.add_argument(
        "--roi",
        nargs="+",
        type=int,
        help="the region of interst which area is actually calculated(x, y, width, height)",
    )
    args = parser.parse_args()

    cam_id = args.cam_id
    port_name = args.port
    max_iter = args.max_iter
    min = args.min
    max = args.max
    tolerance = args.tol
    gray_scale = args.gray
    wait_time = args.wait
    scale = args.scale
    min_interval = args.mit
    ROI = args.roi

    cam = AutofocusCamera(cam_id=cam_id, port_name=port_name, wait_time=wait_time)
    len_val, contrast_measure = cam.focus_cam(
        max_iter,
        min,
        max,
        tolerance,
        ROI=ROI,
        gray_scale=gray_scale,
        scale=scale,
        min_interval=min_interval,
    )

    # check
    while True:
        ret, frame = cam.capture.read()
        cv2.imshow("frame", frame)

        arg = cv2.waitKey(1)
        if arg & 0xFF == ord("q"):
            break

    cam.capture.release()
    cam.ser.close()
    cv2.destroyAllWindows()
