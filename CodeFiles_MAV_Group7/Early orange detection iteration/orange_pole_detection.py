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

def filter_color(image_name = 'imagefolder/52083043.jpg', y_low = 50, y_high = 240, u_low = 0, u_high = 100, v_low = 170, v_high = 255, resize_factor=1):
    
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

def draw_rect(image, column, left_edge):
    top = 0
    bottom = 0
    left = 0
    right = 0
    
    top_found = False
    bottom_found = False
    left_found = False
    right_found = False
    
    threshold = 14
    
    counter = 0
    
    while top_found == False:
        if np.count_nonzero(column[counter]) > threshold:
            top_found = True
            top = counter
        counter+=1
        if counter==240:
            top_found = True
            top = 238
        
    counter = len(column[:,0]) - 1
    
    while bottom_found == False:
        if np.count_nonzero(column[counter]) > threshold:
            bottom_found = True
            bottom = counter
        counter-=1
        if counter==0:
            bottom_found = True
            bottom = 0
    
    
    counter = 0
    
    while left_found == False:
        if np.count_nonzero(column[:,counter]) > threshold:
            left_found = True
            left = counter
        counter+=1    


    counter = len(column[0,:]) - 1
    
    while right_found == False:
        if np.count_nonzero(column[:,counter]) > threshold:
            right_found = True
            right = counter
        counter-=1
    
    
    image[top,   left_edge+left : left_edge+right] = 2
    image[top+1, left_edge+left : left_edge+right] = 2
    
    image[bottom,   left_edge+left : left_edge+right] = 2
    image[bottom-1, left_edge+left : left_edge+right] = 2
    
    image[top:bottom,   left_edge+left] = 2
    image[top:bottom, left_edge+left+1] = 2
    
    image[top:bottom,   left_edge+right] = 2
    image[top:bottom, left_edge+right-1] = 2
    
    return top, bottom

def analyse_image(image):
    image_width = len(image[0,:])
    image_height = len(image[:,0])
    
    top_value = 10
    bot_value = 233
    
    width = int(image_width/5)
    
    column1 = image[:,width*0:width*1]
    column2 = image[:,width*1:width*2]
    column3 = image[:,width*2:width*3]
    column4 = image[:,width*3:width*4]
    column5 = image[:,width*4:width*5]
    
    total_column_pixels = image_height*width

    threshold_nearby_obstacle_detected = 0.10
    
    action = []
    
    if np.count_nonzero(column1) > total_column_pixels*threshold_nearby_obstacle_detected:
        top, bot = draw_rect(image, column1, width*0)
        if top > top_value and bot < bot_value:
            action.append("1 object is to the left-most side of the screen but relatively far, keep going.")
        if top > top_value and bot > bot_value:    
            action.append("1 object is to the left-most side of the screen and pretty close, roll slightly right.")
        if top < top_value and bot > bot_value:
            action.append("1 object is to the left-most side of the screen and very close, roll right.")
            
        
    if np.count_nonzero(column2) > total_column_pixels*threshold_nearby_obstacle_detected:
        top, bot = draw_rect(image, column2, width*1)
        if top > top_value and bot < bot_value:
            action.append("1 object is center-left of the screen but relatively far, keep going.")
        if top > top_value and bot > bot_value:    
            action.append("1 object is center-left of the screen and pretty close, roll slightly right.")
        if top < top_value and bot > bot_value:
            action.append("1 object is center-left of the screen and very close, slow down and turn right.")
    
    if np.count_nonzero(column3) > total_column_pixels*threshold_nearby_obstacle_detected:
        top, bot = draw_rect(image, column3, width*2)
        if top > top_value and bot < bot_value:
            action.append("1 object is dead center of the screen but relatively far, course adjustment requirement if course is maintained.")
        if top > top_value and bot > bot_value:    
            action.append("1 object is dead center of the screen and pretty close, slow down and turn either right or left.")
        if top < top_value and bot > bot_value:
            action.append("1 object is dead center of the screen and very close. Stop immediately and turn either right or left")

    if np.count_nonzero(column4) > total_column_pixels*threshold_nearby_obstacle_detected:
        top, bot = draw_rect(image, column4, width*3)
        if top > top_value and bot < bot_value:
            action.append("1 object is center-right of the screen but relatively far, keep going.")
        if top > top_value and bot > bot_value:    
            action.append("1 object is center-right of the screen and pretty close, roll slightly left.")
        if top < top_value and bot > bot_value:
            action.append("1 object is center-right of the screen and very close, slow down and turn left.")

    if np.count_nonzero(column5) > total_column_pixels*threshold_nearby_obstacle_detected:
        top, bot = draw_rect(image, column5, width*4)
        if top > top_value and bot < bot_value:
            action.append("1 object is to the right-most side of the screen but relatively far, keep going.")
        if top > top_value and bot > bot_value:    
            action.append("1 object is to the right-most side of the screen and pretty close, roll slightly left.")
        if top < top_value and bot > bot_value:
            action.append("1 object is to the right-most side of the screen and very close, roll left.")
            
    title = ""
    for i in action:
        title += i+"\n"

    return image, title

for i in os.listdir('imagefolder'):
    start = time.time()
    image_original, filtered = filter_color(image_name = 'imagefolder/'+i, y_low = 50, y_high = 240, u_low = 0, u_high = 130, v_low = 170, v_high = 255);
    image_result, title = analyse_image(filtered)
    end = time.time()
    
    total_time = end-start
    
    plt.figure()    
    title+="Calculated in "+str(round(total_time,5))+" seconds"
    
    plt.subplot(121)
    RGB = cv2.cvtColor(image_original, cv2.COLOR_BGR2RGB);
    plt.imshow(RGB)
    
    plt.subplot(122)
    plt.imshow(image_result);
    
    plt.suptitle(title);
    
    title+="Calculated in "+str(round(total_time,5))+" seconds"
    
