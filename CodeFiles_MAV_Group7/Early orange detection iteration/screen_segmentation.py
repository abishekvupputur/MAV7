# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import cv2;
import numpy as np;
import matplotlib.pyplot as plt
import time
import os

def YUVSplit(image):
    [Y, U, V] = np.dsplit(image,image.shape[-1])
    Y = Y.flatten().reshape(image.shape[0],image.shape[1])
    U = U.flatten().reshape(image.shape[0],image.shape[1])
    V = V.flatten().reshape(image.shape[0],image.shape[1])
    return Y, U, V

def filter_orange(image_name = 'imagefolder/52083043.jpg', y_low = 50, y_high = 240, u_low = 0, u_high = 100, v_low = 170, v_high = 255, resize_factor=1):
    
    im = cv2.imread(image_name);
    im = cv2.resize(im, (int(im.shape[1]/resize_factor), int(im.shape[0]/resize_factor)));
    YUV = cv2.cvtColor(im, cv2.COLOR_BGR2YUV);
    
    [Y,U,V] = YUVSplit(YUV)
    
    Y = np.where((Y >= y_low) & (Y <= y_high), 1, 0)
    U = np.where((U >= u_low) & (U <= u_high), 1, 0)
    V = np.where((V >= v_low) & (V <= v_high), 1, 0)

    filtered = Y+U+V
    filtered = np.where(filtered == 3, 1, 0)
    filtered = np.rot90(filtered, 1)
    
    return np.rot90(im,1), filtered

def filter_green(image_name = 'imagefolder/52083043.jpg', y_low = 50, y_high = 240, u_low = 0, u_high = 100, v_low = 170, v_high = 255, resize_factor=1):
    
    im = cv2.imread(image_name);
    im = cv2.resize(im, (int(im.shape[1]/resize_factor), int(im.shape[0]/resize_factor)));
    YUV = cv2.cvtColor(im, cv2.COLOR_BGR2YUV);
    
    [Y,U,V] = YUVSplit(YUV)
    
    Y1 = Y[:,0:120]
    Y2 = Y[:,120:240]
    
    U1 = U[:,0:120]
    U2 = U[:,120:240]
    
    V1 = V[:,0:120]
    V2 = V[:,120:240]
    
    Y1 = np.where((Y1 >= y_low) & (Y1 <= y_high), 1, 0)
    U1 = np.where((U1 >= u_low) & (U1 <= u_high), 1, 0)
    V1 = np.where((V1 >= v_low) & (V1 <= v_high), 1, 0)
   
    Y2 = np.zeros(np.shape(Y2))
    U2 = np.zeros(np.shape(U2))
    V2 = np.zeros(np.shape(V2))
    
    Y = np.hstack((Y1, Y2))
    U = np.hstack((U1, U2))
    V = np.hstack((V1, V2))


    filtered = Y+U+V
    filtered = np.where(filtered == 3, 2, 0)
    filtered = np.rot90(filtered, 1)
    
    
    return np.rot90(im,1), filtered

for i in os.listdir('imagefolder'):
    
    image_original, filtered_orange = filter_orange(image_name = 'imagefolder/'+i, y_low = 50, y_high = 240, u_low = 0, u_high = 130, v_low = 170, v_high = 255);
    
    image_original, filtered_green = filter_green(image_name = 'imagefolder/'+i, y_low = 70, y_high = 100, u_low = 90, u_high = 130, v_low = 0, v_high = 135);
    
    filtered = filtered_green+filtered_orange
    filtered[:,104] = 3
    filtered[:,208] = 3
    filtered[:,312] = 3
    filtered[:,416] = 3
    filtered[120,:] = 4
    plt.figure()    
    plt.subplot(121)
    RGB = cv2.cvtColor(image_original, cv2.COLOR_BGR2RGB);
    plt.imshow(RGB)
    plt.subplot(122)
    plt.imshow(filtered);