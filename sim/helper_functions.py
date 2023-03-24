#!/usr/bin/python3

# HELPER FUNCTIONS
import numpy as np
import cv2 as cv


def diff_angle(angle1, angle2):
    return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))


const_verysmall = 3541/15.0
M_R2L = np.array([[1.0, 0.0], [0.0, -1.0]])
T_R2L = np.array([0, 15.0])


def mL2pix(ml):
    # meters to pixel (left frame)
    return np.int32(ml*const_verysmall)


def pix2mL(pix):
    # pixel to meters (left frame)
    return pix/const_verysmall


def mL2mR(m):
    # meters left frame to meters right frame
    return m @ M_R2L + T_R2L


def mR2mL(m):
    # meters right frame to meters left frame
    return (m - T_R2L) @ M_R2L


def mR2pix(mr):
    # meters to pixel (right frame), return directly a cv point
    if mr.size == 2:
        pix = mL2pix(mR2mL(mr))
        return (pix[0], pix[1])
    else:
        return mL2pix(mR2mL(mr))


def pix2mR(pix):
    # pixel to meters (right frame)
    return mL2mR(pix2mL(pix))


def draw_car(track, x, y, angle, color=(0, 255, 0),  draw_body=True):
    # function to draw the car on the track
    car_length = 0.45-0.22  # m
    car_width = 0.2  # m
    # match angle with track frame of reference
    # angle = yaw2world(angle)
    # find 4 corners not rotated car_width
    corners = np.array([[-0.22, car_width/2],
                        [car_length, car_width/2],
                        [car_length, -car_width/2],
                        [-0.22, -car_width/2]])
    # rotate corners
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]])
    corners = corners @ rot_matrix.T
    # add car position
    corners = corners + np.array([x, y])
    # draw body
    if draw_body:
        cv.polylines(track, [mR2pix(corners)], True, color, 3, cv.LINE_AA)
    return track


def project_onto_frame(frame, car, points, align_to_car=True,
                       color=(0, 255, 255), thickness=2):
    # check if its a single point
    single_dim = False
    if points.ndim == 1:
        points = points[np.newaxis, :]
        single_dim = True
    num_points = points.shape[0]
    # check shape
    if points[0].shape == (2,):
        # add 0 Z component
        points = np.concatenate((points, -car.CAM_Z*np.ones((num_points, 1))),
                                axis=1)

    # rotate the points around the z axis
    if align_to_car:
        points_cf = to_car_frame(points, car, 3)
    else:
        points_cf = points

    angles = np.arctan2(points_cf[:, 1], points_cf[:, 0])
    # calculate angle differences
    diff_angles = diff_angle(angles, 0.0)  # car.yaw
    # get points in front of the car
    rel_pos_points = []
    for i, point in enumerate(points):
        if np.abs(diff_angles[i]) < car.CAM_FOV/2:
            # front_points.append(point)
            rel_pos_points.append(points_cf[i])

    # convert to numpy
    rel_pos_points = np.array(rel_pos_points)

    if len(rel_pos_points) == 0:
        # return frame, proj
        return frame, None

    # add diffrence com to back wheels
    rel_pos_points = rel_pos_points - np.array([0.18, 0.0, 0.0])

    # rotate the points around the relative y axis, pitch
    beta = -car.CAM_PITCH
    rot_matrix = np.array([[np.cos(beta), 0, np.sin(beta)],
                           [0, 1, 0],
                           [-np.sin(beta), 0, np.cos(beta)]])

    rotated_points = rel_pos_points @ rot_matrix.T

    # project the points onto the camera frame
    proj_points = np.array([[-p[1]/p[0], -p[2]/p[0]] for p in rotated_points])
    # convert to pixel coordinates
    proj_points = 240*proj_points + np.array([320//2, 240//2])  # 320x240
    # draw the points
    for i in range(proj_points.shape[0]):
        p = proj_points[i]
        assert p.shape == (2,), f"projection point has wrong shape: {p.shape}"
        # print(f'p = {p}')
        p1 = (int(round(p[0])), int(round(p[1])))
        # print(f'p = {p}')
        # check if the point is in the frame
        if p1[0] >= 0 and p1[0] < 320 and p1[1] >= 0 and p1[1] < 240:
            try:
                cv.circle(frame, p1, thickness, color, -1)
            except Exception as e:
                print(f'Error drawing point {p}')
                print(p1)
                print(e)

    if single_dim:
        return frame, proj_points[0]
    return frame, proj_points


def to_car_frame(points, car, return_size=3):
    # check if its a single point
    single_dim = False
    if points.ndim == 1:
        points = points[np.newaxis, :]
        single_dim = True
    gamma = car.yaw
    if points.shape[1] == 3:
        points_cf = points - np.array([car.x_true, car.y_true, 0])
        rot_matrix = np.array([[np.cos(gamma), -np.sin(gamma), 0],
                               [np.sin(gamma), np.cos(gamma), 0],
                               [0, 0, 1]])
        out = points_cf @ rot_matrix
        if return_size == 2:
            out = out[:, :2]
        assert out.shape[1] == return_size, \
            "wrong size, got {}".format(out.shape[1])
    elif points.shape[1] == 2:
        points_cf = points - np.array([car.x_true, car.y_true])
        rot_matrix = np.array([[np.cos(gamma), -np.sin(gamma)],
                               [np.sin(gamma), np.cos(gamma)]])
        out = points_cf @ rot_matrix
        if return_size == 3:
            out = np.concatenate((out, np.zeros((out.shape[0], 1))), axis=1)
        assert out.shape[1] == return_size, \
            "wrong size, got {}".format(out.shape[1])
    else:
        raise ValueError("points must be (2,), or (3,)")
    if single_dim:
        return out[0]
    else:
        return out


def draw_bounding_box(frame, bounding_box, color=(0, 0, 255)):
    x, y, x2, y2 = bounding_box
    x, y, x2, y2 = round(x), round(y), round(x2), round(y2)
    cv.rectangle(frame, (x, y), (x2, y2), color, 2)
    return frame


def get_curvature(points, v_des=0.0):
    diff = points[1:] - points[:-1]
    distances = np.linalg.norm(diff, axis=1)
    d = np.mean(distances)
    angles = np.arctan2(diff[:, 1], diff[:, 0])
    alphas = diff_angle(angles[1:], angles[:-1])
    alpha = np.mean(alphas)
    curv = (2*np.sin(alpha*0.5)) / d
    COMPENSATION_FACTOR = 0.855072
    return curv * COMPENSATION_FACTOR


def wrap_detection(output_data):
    # detection functions
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
                x, y = row[0].item(), row[1].item()
                w, h = row[2].item(), row[3].item()
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
    return np.exp(x) / np.sum(np.exp(x), axis=0)


def project_curvature(frame, car, curv):
    color = (0, 255, 0)
    start_from = 0.13
    d_ahead = 0.4  # [m]
    num_points = 20
    # multiply by constant, to be tuned
    curv = curv * 1.0  # 30.
    # get radius from curvature
    r = 1. / curv
    print("r: {}".format(r))
    x = np.linspace(start_from, start_from + d_ahead, num_points)
    y = - np.sqrt(r**2 - x**2) * np.sign(curv) + r
    # stack x and y into one array
    points = np.stack((x, y), axis=1)
    # project points onto car frame, note: they are already inn car frame
    frame, proj_points = project_onto_frame(frame=frame,
                                            car=car,
                                            points=points,
                                            align_to_car=False,
                                            color=color)
    # draw a line connecting the points
    if proj_points is not None:
        # convert proj to int32
        proj_points = proj_points.astype(np.int32)
        cv.polylines(frame, [proj_points], False, color, 2)
    return r


def project_stopline(frame, car, stopline_x, stopline_y,
                     car_angle_to_stopline, color=(0, 200, 0)):
    points = np.zeros((50, 2), dtype=np.float32)
    points[:, 1] = np.linspace(-0.19, 0.19, 50)

    slp_cf = np.array([stopline_x+0.35, stopline_y])

    rot_matrix = np.array([[np.cos(car_angle_to_stopline),
                            -np.sin(car_angle_to_stopline)],
                           [np.sin(car_angle_to_stopline),
                            np.cos(car_angle_to_stopline)]])
    points = points @ rot_matrix  # rotation
    points = points + slp_cf  # translation

    frame, proj_points = project_onto_frame(frame=frame,
                                            car=car,
                                            points=points,
                                            align_to_car=False,
                                            color=color)
    return frame, proj_points


def get_yaw_closest_axis(angle):
    """
    Returns the angle multiple of pi/2 closest to the given angle
    e.g. returns one of these 4 possible options: [-pi/2, 0, pi/2, pi]
    """
    int_angle = round(angle/(np.pi/2))
    assert int_angle in [-2, -1, 0, 1, 2], f'angle: {int_angle}'
    if int_angle == -2:
        int_angle = 2
    return int_angle*np.pi/2


def m2pix(m):
    return np.int32(m*const_verysmall)


def pix2m(pix):
    return 1.0*pix/const_verysmall


def yaw2world(angle):
    return -(angle + np.pi/2)


def world2yaw(angle):
    return -angle - np.pi/2


def create_frames(show):
    if show:
        cv.namedWindow('frame', cv.WINDOW_NORMAL)
        cv.resizeWindow('frame', 640, 480)
        # show windows
        cv.namedWindow('Path', cv.WINDOW_NORMAL)
        cv.resizeWindow('Path', 600, 600)
        cv.namedWindow('Map', cv.WINDOW_NORMAL)
        cv.resizeWindow('Map', 600, 600)


def show_track(track, car, show=True):
    if show:
        track1 = track.copy()
        draw_car(track1, car.x_true, car.y_true, car.yaw_true)
        cv.imshow('Map', track1)
        cv.waitKey(1)


def show_car(track, car, brain, show=True):
    if show:
        track1 = track.copy()
        draw_car(track1, car.x, car.y, car.yaw, color=(180, 0, 0))
        if car.trust_gps:
            color = (255, 0, 255)
        else:
            color = (100, 0, 100)
        draw_car(track1, car.x_true, car.y_true,
                 car.yaw_true, color=(0, 180, 0))
        draw_car(track1, car.x_est, car.y_est, car.yaw, color=color)
        if len(brain.path_planner.path) > 0:
            cv.circle(track1, mR2pix(brain.path_planner.path[int(
                brain.car_dist_on_path*100)]), 10, (150, 50, 255), 3)
        cv.imshow('Map', track1)
        cv.waitKey(1)


def show_camera(car, brain, show=True):
    if show:
        frame = car.frame.copy()
        cv.imshow('frame', frame)


def show_local_path_just_switched(brain,
                                  alpha,
                                  stop_line_yaw,
                                  car_position_slf,
                                  local_path_cf,
                                  stop_line_position,
                                  show=True):
    if show:
        img = brain.car.frame.copy()
        # project the whole path (true)
        img, _ = project_onto_frame(img, brain.car,
                                    brain.path_planner.path,
                                    align_to_car=True,
                                    color=(0, 100, 0))
        # project local path (estimated), it should match the true path
        img, _ = project_onto_frame(img, brain.car, local_path_cf,
                                    align_to_car=False)
        cv.imshow('brain_debug', img)
        cv.waitKey(1)
        # var2 hold original position
        brain.curr_state.var2 = np.array([brain.car.x_true,
                                         brain.car.y_true])
        true_start_pos_wf = brain.curr_state.var2

        alpha = alpha + stop_line_yaw
        rot_matrix = np.array([[np.cos(alpha), -np.sin(alpha)],
                              [np.sin(alpha), np.cos(alpha)]])

        est_car_pos_slf = car_position_slf
        est_car_pos_slf_rot = est_car_pos_slf @ rot_matrix.T
        est_car_pos_wf = est_car_pos_slf_rot + stop_line_position
        cv.circle(brain.path_planner.map, mR2pix(est_car_pos_wf),
                  25, (255, 0, 255), 5)
        cv.circle(brain.path_planner.map, mR2pix(true_start_pos_wf),
                  30, (0, 255, 0), 5)
        cv.imshow('Path', brain.path_planner.map)
        cv.waitKey(1)

        cv.namedWindow('local_path', cv.WINDOW_NORMAL)
        local_map_img = np.zeros_like(brain.path_planner.map)
        h = local_map_img.shape[0]
        w = local_map_img.shape[1]
        local_map_img[w//2-2:w//2+2, :] = 255
        local_map_img[:, h//2-2:h//2+2] = 255

        cv.circle(local_map_img, (w//2, h//2), 50, (255, 0, 255), 5)
        for i in range(len(local_path_cf)):
            if (i % 3 == 0):
                p = local_path_cf[i]
                pix = mR2pix(p)
                pix = (int(pix[0]+w//2), int(pix[1]-h//2))
                cv.circle(local_map_img, pix, 10, (0, 150, 150), -1)
        cv.imshow('local_path', local_map_img)
        cv.waitKey(1)
        brain.curr_state.var3 = local_map_img


def show_local_path(brain, car_pos_loc, show=True):
    if show:
        local_map_img = brain.curr_state.var3
        h = local_map_img.shape[0]
        w = local_map_img.shape[1]
        angle = brain.car.yaw_loc_o  # + brain.car.yaw_loc
        rot_matrix_w = np.array([[np.cos(angle), -np.sin(angle)],
                                 [np.sin(angle), np.cos(angle)]])
        # show car position in the local frame (from the encoder)
        cv.circle(local_map_img, (mR2pix(car_pos_loc)[0]+w//2,
                                  mR2pix(car_pos_loc)[1]-h//2),
                  5, (255, 0, 255), 2)
        # show the true position to check if they match, translated
        # wrt starting position into the local frame
        true_start_pos_wf = brain.curr_state.var2
        true_pos_loc = np.array([brain.car.x_true, brain.car.y_true]) - \
            true_start_pos_wf
        true_pos_loc = true_pos_loc @ rot_matrix_w
        cv.circle(local_map_img, (mR2pix(true_pos_loc)[0]+w//2,
                                  mR2pix(true_pos_loc)[1]-h//2),
                  7, (0, 255, 0), 2)
        cv.imshow('local_path', local_map_img)
        true_start_pos_wf = brain.curr_state.var2
        car_pos_loc_rot_wf = car_pos_loc @ rot_matrix_w.T
        car_pos_wf = true_start_pos_wf + car_pos_loc_rot_wf
        # show car position in wf (encoder)
        cv.circle(brain.path_planner.map, mR2pix(car_pos_wf), 5,
                  (255, 0, 255), 2)
        # show the true position to check if they match
        true_pos_wf = np.array([brain.car.x_true, brain.car.y_true])
        cv.circle(brain.path_planner.map, mR2pix(true_pos_wf), 7,
                  (0, 255, 0), 2)
        cv.imshow('Path', brain.path_planner.map)
        cv.waitKey(1)


def show_brain_debug(brain, local_path_cf, point_ahead, show):
    if show:
        img = brain.car.frame.copy()
        img, _ = project_onto_frame(img, brain.car, local_path_cf,
                                    align_to_car=False)
        img, _ = project_onto_frame(img, brain.car, point_ahead,
                                    align_to_car=False,
                                    color=(0, 0, 255))
        cv.imshow('brain_debug', img)
        cv.waitKey(1)


def show_follow_lane(brain, point_ahead, show):
    if show:
        img = brain.car.frame.copy()
        img, proj = project_onto_frame(img, brain.car, point_ahead,
                                       align_to_car=False,
                                       color=(255, 0, 255),
                                       thickness=3)
        img = cv.line(img, (int(proj[0]), int(proj[1])),
                      (int(img.shape[1]/2), int(img.shape[0])),
                      (255, 0, 255), 2)
        cv.imshow('brain_debug', img)
        cv.waitKey(1)
