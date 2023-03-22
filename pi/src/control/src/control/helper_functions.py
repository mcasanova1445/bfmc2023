#!/usr/bin/python3

# HELPER FUNCTIONS
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import glob
import os
from mpl_toolkits.mplot3d import Axes3D

def diff_angle(angle1, angle2):
    return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))

#const_simple = 196.5
#const_med = 14164/15.0
const_verysmall = 3541/15.0
def m2pix(m):
    return np.int32(m*const_verysmall)

def pix2m(pix):
    return 1.0*pix/const_verysmall

def yaw2world(angle):
        return -(angle + np.pi/2)

def world2yaw(angle):
    return -angle -np.pi/2

#function to draw the car on the map
def draw_car(map, x, y, angle, color=(0, 255, 0),  draw_body=True):
    car_length = 0.4 #m
    car_width = 0.2 #m
    #match angle with map frame of reference
    angle = yaw2world(angle)
    #find 4 corners not rotated
    corners = np.array([[-car_width/2, car_length/2],
                        [car_width/2, car_length/2],
                        [car_width/2, -car_length/2],
                        [-car_width/2, -car_length/2]])
    #rotate corners
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],[np.sin(angle), np.cos(angle)]])
    corners = np.matmul(rot_matrix, corners.T)
    #add car position
    corners = corners.T + np.array([x,y])
    #draw body
    if draw_body:
        cv.polylines(map, [m2pix(corners)], True, color, 3, cv.LINE_AA) 
    return map

def project_onto_frame(frame, car, points, align_to_car=True, color=(0,255,255)):
    #check if its a single point
    single_dim = False
    if points.ndim == 1:
        points = points[np.newaxis,:]
        single_dim = True
    num_points = points.shape[0]
    #check shape
    if points[0].shape == (2,):
        #add 0 Z component
        points = np.concatenate((points, -car.cam_z*np.ones((num_points,1))), axis=1)

    #rotate the points around the z axis
    if align_to_car:
        diff = to_car_frame(points, car, 3)
    else:
        diff = points

    #get points inside the fov:
    #angles
    angles = - np.arctan2(diff[:,1], diff[:,0])
    #calculate angle differences
    diff_angles = diff_angle(angles, 0.0) #car.yaw
    #get points in front of the car
    # front_points = []
    rel_pos_points = []
    for i, point in enumerate(points):
        if np.abs(diff_angles[i]) < car.cam_fov/2:
            # front_points.append(point)
            rel_pos_points.append(diff[i])

    #convert to numpy
    # front_points = np.array(front_points)
    rel_pos_points = np.array(rel_pos_points)

    if len(rel_pos_points) == 0:
        # return frame, proj
        return frame, None
    
    # #plot the points in the 2d map
    # for p in front_points:
    #     cv.circle(map, m2pix(p[:2]), 15, (0,255,255), -1)

    #rotate the points around the relative y axis, pitch
    beta = -car.cam_pitch
    rot_matrix = np.array([[np.cos(beta), 0, np.sin(beta)],
                            [0, 1, 0],
                            [-np.sin(beta), 0, np.cos(beta)]])
    
    rotated_points = np.matmul(rot_matrix, rel_pos_points.T).T

    #project the points onto the camera frame
    proj_points = np.array([[p[1]/p[0], -p[2]/p[0]] for p in rotated_points])
    #convert to pixel coordinates
    proj_points = 490*proj_points + np.array([320, 240])

    # draw the points
    for p in proj_points:
        cv.circle(frame, (round(p[0]), round(p[1])), 2, color, -1)

    if single_dim:
        return frame, proj_points[0]
    return frame, proj_points

def to_car_frame(points, car, return_size=3):
    #check if its a single point
    single_dim = False
    if points.ndim == 1:
        points = points[np.newaxis,:]
        single_dim = True
    gamma = car.yaw
    if points.shape[1] == 3:
        diff = points - np.array([car.x_true, car.y_true, 0])
        rot_matrix = np.array([[np.cos(gamma), -np.sin(gamma), 0],[np.sin(gamma), np.cos(gamma), 0 ], [0,0,1]])
        out = np.matmul(rot_matrix, diff.T).T
        if return_size == 2:
            out = out[:,:2]
        assert out.shape[1] == return_size, "wrong size, got {}".format(out.shape[1])
    elif points.shape[1] == 2:
        diff = points - np.array([car.x_true, car.y_true]) 
        rot_matrix = np.array([[np.cos(gamma), -np.sin(gamma)],[np.sin(gamma), np.cos(gamma)]])
        out = np.matmul(rot_matrix, diff.T).T
        if return_size == 3:
            out = np.concatenate((out, np.zeros((out.shape[0],1))), axis=1)
        assert out.shape[1] == return_size, "wrong size, got {}".format(out.shape[1])
    else: raise ValueError("points must be (2,), or (3,)")
    if single_dim: return out[0]
    else: return out

def draw_bounding_box(frame, bounding_box, color=(0,0,255)):
    x,y,x2,y2 = bounding_box
    x,y,x2,y2 = round(x), round(y), round(x2), round(y2)
    cv.rectangle(frame, (x,y), (x2,y2), color, 2)
    return frame

def get_curvature(points, v_des):
    # calculate curvature 
    local_traj = points
    #get length
    path_length = 0
    for i in range(len(points)-1):
        x1,y1 = points[i]
        x2,y2 = points[i+1]
        path_length += np.hypot(x2-x1,y2-y1) 
    #time
    tot_time = path_length / v_des
    local_time = np.linspace(0, tot_time, len(local_traj))
    dx_dt = np.gradient(local_traj[:,0], local_time)
    dy_dt = np.gradient(local_traj[:,1], local_time)
    dp_dt = np.gradient(local_traj, local_time, axis=0)
    v = np.linalg.norm(dp_dt, axis=1)
    ddx_dt = np.gradient(dx_dt, local_time)
    ddy_dt = np.gradient(dy_dt, local_time)
    curv = (dx_dt*ddy_dt-dy_dt*ddx_dt) / np.power(v,1.5)
    avg_curv = np.mean(curv)
    return avg_curv

#detection functions
def wrap_detection(output_data):
    class_ids = []
    confidences = []
    boxes = []
    rows = output_data.shape[0]
    for r in range(rows):
        row = output_data[r]
        confidence = row[4]
        if confidence >= 0.3:
            classes_scores = row[5:]
            _, _, _, max_indx = cv.minMaxLoc(classes_scores)
            class_id = max_indx[1]
            if (classes_scores[class_id] > .25):
                confidences.append(confidence)
                class_ids.append(class_id)
                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                left = int((x - 0.5 * w))
                top = int((y - 0.5 * h))
                width = int(w)
                height = int(h)
                box = np.array([left, top, width, height])
                boxes.append(box)

    indexes = cv.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 
    result_class_ids = []
    result_confidences = []
    result_boxes = []
    for i in indexes:
        result_confidences.append(confidences[i])
        result_class_ids.append(class_ids[i])
        result_boxes.append(boxes[i])
    return result_class_ids, result_confidences, result_boxes

def my_softmax(x):
    x = x.reshape(-1, 1)
    return np.exp(x) / np.sum(np.exp(x), axis=0)
    y = np.exp(x - np.max(x), axis=1)
    f_x = y / np.sum(np.exp(x))
    return f_x


def mR2mL(m): #meters right frame to meters left frame
    return (m - T_R2L) @ M_R2L 






