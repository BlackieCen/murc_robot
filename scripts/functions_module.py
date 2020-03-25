#!/usr/bin/env python
# --------------------------------------
# file:      functions_module.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     parameter_adjustment ---> adjust the parameter in thresholding
#            coordinates_3D_calculation ---> transforming the 2D coordinatens (u,v) to 3D (xyz)
#            box2rect ---> calculate the width and height of the rectangle
# --------------------------------------------------

import cv2
import numpy as np


def parameter_adjustment(src_img, para_num, function_to_adjust):
    pa_image_win = "Image after Parameter Adjustment for " + function_to_adjust
    cv2.namedWindow(pa_image_win, cv2.WINDOW_AUTOSIZE)
    # parameters'range and initial values
    if function_to_adjust == "adaptiveThreshold":
        cv2.createTrackbar("para1", pa_image_win, 7, 255, nothing)
        cv2.createTrackbar("para2", pa_image_win, 3, 10, nothing)
    elif function_to_adjust == "GaussianBlur":
        for idx in range(1, para_num + 1):
            para_name = "para" + str(idx)
            cv2.createTrackbar(para_name, pa_image_win, 2, 10, nothing)
    else:
        for idx in range(1, para_num + 1):
            para_name = "para" + str(idx)
            cv2.createTrackbar(para_name, pa_image_win, 60, 255, nothing)

    while True:
        parameters = [0]
        for i in range(1, para_num + 1):
            parameters.append(cv2.getTrackbarPos("para" + str(i), pa_image_win))
        # insert the function to apply in and limits of parameters
        if function_to_adjust == "adaptiveThreshold":
            parameters[1] = odd_judgment(parameters[1])
            pa_image = cv2.adaptiveThreshold(src_img, 255,
                                             cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                             cv2.THRESH_BINARY_INV,
                                             parameters[1], parameters[2])  # blockSize , C
        elif function_to_adjust == "threshold":
            ret, pa_image = cv2.threshold(src_img, parameters[1], 255, cv2.THRESH_BINARY)
        elif function_to_adjust == "Canny":
            pa_image = cv2.Canny(src_img, parameters[1], parameters[2])
        elif function_to_adjust == "GaussianBlur":
            parameters[1] = odd_judgment(parameters[1])
            parameters[2] = odd_judgment(parameters[2])
            pa_image = cv2.GaussianBlur(src_img, (parameters[1], parameters[2]), parameters[3], parameters[4])
        # end of the function
        cv2.imshow(pa_image_win, pa_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return pa_image


def nothing(x):
    pass


def odd_judgment(para):
    if para % 2 == 0:
        para = para - 1
    if para < 3:
        para = 3
    return para


# # Remove background - Set pixels further than clipping_distance to grey
# def remove_background(clipping_distance_in_meters, distance_factor, depth_image ):
#     clipping_distance = clipping_distance_in_meters/distance_factor
#     gray_color = 153 # gray_rgb = (153,153,153)


def coordinates_3D_calculation(u, v, depth_img, camera_stream):
    # camera_factor = depth_scale
    if camera_stream == "depth70":
        # intrinsics of depth camera 70
        camera_cx = 641.227
        camera_cy = 354.445
        camera_fx = 650.822
        camera_fy = 650.822
    elif camera_stream == "depth75":
        # intrinsics of depth camera 75 848x480
        camera_cx = 424.813
        camera_cy = 236.32
        camera_fx = 431.169
        camera_fy = 431.169
    elif camera_stream == "aligned_depth":
        camera_cx = 424.000
        camera_cy = 237.855
        camera_fx = 615.718
        camera_fy = 616.188
    # # intrinsics of color camera
    # camera_cx = 639.999
    # camera_cy = 356.782
    # camera_fx = 923.577
    # camera_fy = 924.282
    vmax = depth_img.shape[0]
    umax = depth_img.shape[1]
    if u < 0:
        u = 0
    if u >= umax:
        u = umax -1
    if v < 0:
        v = 0
    if v >= vmax:
        v = vmax -1
    depth_in_meters = depth_img[v][u]
    # if depth_in_meters * 1000 < 1:
    #     depth_img_uv_10x10 = depth_img[(v-3):(v+3),(u-3):(u+3)]
    #     if depth_img_uv_10x10 is not None:
    #         max = np.max(depth_img_uv_10x10)
    #         if max * 1000 > 1:
    #             depth_in_meters = max
    if depth_in_meters * 1000 < 1:
        print("WARNING: depth image in point({},{}) is {}".format(u, v, depth_in_meters))
        coordinates = [0,0,0]
    else:
        z = float(depth_in_meters)
        x = (u - camera_cx) * z / camera_fx
        y = (v - camera_cy) * z / camera_fy
        coordinates = [x, y, z]

    # M = np.array([[camera_fx, 0, camera_cx], [0, camera_fy, camera_cy], [0, 0, 1]])
    # M_inv = np.linalg.inv(M)
    # coordinates_pixel = np.array([[u], [v], [1]])
    # coordinates = np.dot(M_inv, coordinates_pixel) * z

    return coordinates


def get_distance(point1, point2):
    distance = pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2) + pow(point1[2] - point2[2], 2)
    distance = np.sqrt(distance)
    return distance


def box2rect(box_4points):
    if box_4points.shape == (4, 2):
        b = box_4points
        center = tuple(np.sum(b, axis=0)/4)
        width = (np.sqrt(pow(b[1][0] - b[2][0], 2) + pow(b[1][1] - b[2][1], 2)) +
                 np.sqrt(pow(b[0][0] - b[3][0], 2) + pow(b[0][1] - b[3][1], 2)))/2
        height = (np.sqrt(pow(b[1][0] - b[0][0], 2) + pow(b[1][1] - b[0][1], 2)) +
                  np.sqrt(pow(b[2][0] - b[3][0], 2) + pow(b[2][1] - b[3][1], 2)))/2
        dx = b[0][0] - b[1][0]
        dy = b[0][1] - b[1][1]
        if dx == 0 or dy == 0:
            theta = -90.0
        else:
            theta = -float(cv2.fastAtan2(dx,dy))
        if theta >= 0 or theta < -90:
            print("Error in  function_module box2rect")
        return center, (width, height), theta
    else:
        pass
