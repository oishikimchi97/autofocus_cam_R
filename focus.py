import cv2
import numpy
import glob
import numpy as np
import time
import argparse
import os

def cal_contrast(img, ROI=None):
    contrast = 0
    dst = img.astype('int16')
    if ROI==None:
        ROI=[0, 0, img.shape[1], img.shape[0]]
    xs, ys = ROI[0], ROI[1]
    width, height = ROI[2], ROI[3]
    dstROI = dst[ys:ys+height, xs:xs+width]
    dstRoll = np.roll(dstROI, 1, axis=0)
    contrast_arr = np.square((dstROI - dstRoll)[1:])
    contrast = np.sum(contrast_arr)
    # contrast_per_pixel = float(contrast) / float(np.size(img))
    return contrast

def make_fibonacci(N):
    if N is 1:
        return [0]
    elif N is 2:
        return [0, 1]
    
    fibonacci_list = [0, 1]
    for i in range(2, N):
        fib = fibonacci_list[i-1] + fibonacci_list[i-2]
        fibonacci_list.append(fib)
    return fibonacci_list

def fibonacci_search(img_list, N, a, b, tolerance, ROI=None):
    contrast_list = []
    fibonacci_list = make_fibonacci(N)
    
    x, y = a, b
    img_x = cv2.imread(img_list[x], cv2.IMREAD_COLOR)
    img_y = cv2.imread(img_list[y], cv2.IMREAD_COLOR)
    dst_x = cv2.cvtColor(img_x, cv2.COLOR_BGR2GRAY)
    dst_y = cv2.cvtColor(img_y, cv2.COLOR_BGR2GRAY)
    fx = cal_contrast(dst_x, ROI)
    fy = cal_contrast(dst_y, ROI)
    error = abs(fx - fy)
    
    tolerance_per_pixel = tolerance / np.size(img_x)

    for i in range(N-2):
        L = y - x
        if error < tolerance_per_pixel:
            i = i - 1
            break
            
        print('-'*50)
        print(f'\niteration: {i} ')
        print(f'[x, y]: [{x}, {y}]')
        print(f'error: {error}')
        
        x1 = int(x + (fibonacci_list[N-i-3] / fibonacci_list[N-i-1] ) * L)
        y1 = int(y - (fibonacci_list[N-i-3] / fibonacci_list[N-i-1] ) * L)
        img_x1 = cv2.imread(img_list[x1], cv2.IMREAD_COLOR)
        img_y1 = cv2.imread(img_list[y1], cv2.IMREAD_COLOR)
        dst_x1 = cv2.cvtColor(img_x1, cv2.COLOR_BGR2GRAY)
        dst_y1 = cv2.cvtColor(img_y1, cv2.COLOR_BGR2GRAY)
        fx1 = cal_contrast(dst_x1, ROI)
        fy1 = cal_contrast(dst_y1, ROI)

        if fx1 < fy1:
            print(f'x is changed {x} -> {x1}\n')  
            x = x1
            fx = fx1
            
        else:
            print(f'y is changed {y} -> {y1}\n')
            y = y1
            fy = fy1
        error = abs(fx - fy)
        
    print('Search is finished!')
    print('-'*10)
    print(f'total iteration: {i+1} ')
    print(f'[x, y]: [{x}, {y}]')
    print(f'final error: {error}\n\n')
    if fx > fy:
        print(f'maximum value: {fx} ,index: {x}')
        return x, fx
    else:
        print(f'maximum value: {fy} ,index: {y}')
        return y, fy


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='this code is written for finding a infocus image in folder')

    parser.add_argument('--f', required='True',
                        help='the folder path which include in image datasets')
    parser.add_argument('--r', nargs='+', type=int,
                        help='the region of interst which area is actually calculated(x, y, width, height)')

    args = parser.parse_args()
    folder_path = args.f
    ROI = args.r

    imgs_path = os.path.join(folder_path, '*.bmp')
    img_list = sorted(glob.glob(imgs_path))
    
    N = 20
    tolerance = 2e+6 

    start = time.time()
    max_index, max_fm = fibonacci_search(img_list, 20, 0, 
                                        len(img_list)-1, tolerance, ROI)
    print(f'excution time: {(time.time() - start):.3f}s')
    print(f'the infocus file name: {img_list[max_index]}')
