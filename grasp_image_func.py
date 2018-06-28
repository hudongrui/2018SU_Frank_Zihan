"""
author: Bruce Wingo
dataModified: 12/19/2017
Doc: grasp illustration function 'drawRectangle' and grasp predictor function 'predictGraspOnImage'
    TODO: io to function need to be modified to allow more inputs
    Warning: 'grasp_learner.py' modified at line 42 for continuous prediction with video stream.

"""

import cv2
import numpy as np
from grasp_learner import grasp_obj
from grasp_predictor import Predictors
import time


def drawRectangle(I, h, w, t, gsize=300):
    I_temp = I
    grasp_l = gsize / 2.5
    grasp_w = gsize / 5.0
    grasp_angle = t * (np.pi / 18) - np.pi / 2

    points = np.array([[-grasp_l, -grasp_w],
                       [grasp_l, -grasp_w],
                       [grasp_l, grasp_w],
                       [-grasp_l, grasp_w]])
    R = np.array([[np.cos(grasp_angle), -np.sin(grasp_angle)],
                  [np.sin(grasp_angle), np.cos(grasp_angle)]])
    rot_points = np.dot(R, points.transpose()).transpose()
    im_points = rot_points + np.array([w, h])
    cv2.line(I_temp, tuple(im_points[0].astype(int)), tuple(
        im_points[1].astype(int)), color=(0, 255, 0), thickness=5)
    cv2.line(I_temp, tuple(im_points[1].astype(int)), tuple(
        im_points[2].astype(int)), color=(0, 0, 255), thickness=5)
    cv2.line(I_temp, tuple(im_points[2].astype(int)), tuple(
        im_points[3].astype(int)), color=(0, 255, 0), thickness=5)
    cv2.line(I_temp, tuple(im_points[3].astype(int)), tuple(
        im_points[0].astype(int)), color=(0, 0, 255), thickness=5)
    return I_temp


def predictGraspOnImage(I):
    # default parameters
    model_path = '/home/team18/Grasp-Detector-master/models/Grasp_model'  # 'Grasp model you want to use'
    nsamples = 128  # 'Number of patch samples. More the better, but it\'ll get slower'
    nbest = 1  # 'Number of grasps to display'
    gscale = 0.234375  # 'Scale of grasp. Default is the one used in the paper, given a 720X1280 res image'
    imsize = max(I.shape[:2])
    gsize = int(gscale * imsize)  # Size of grasp patch
    # max_batchsize = 128
    gpu_id = 0  # 'GPU device id; -1 for cpu'

    # Set up model
    batchsize = nsamples
    nbatches = 1

    print('Loading grasp model')
    st_time = time.time()
    G = grasp_obj(model_path, gpu_id)
    G.BATCH_SIZE = batchsize
    G.test_init()
    P = Predictors(I, G)
    print('Time taken: {}s'.format(time.time() - st_time))

    fc8_predictions = []
    patch_Hs = []
    patch_Ws = []
    print('Predicting on samples')
    st_time = time.time()
    for _ in range(nbatches):
        P.graspNet_grasp(patch_size=gsize, num_samples=batchsize)
        fc8_predictions.append(P.fc8_norm_vals)
        patch_Hs.append(P.patch_hs)
        patch_Ws.append(P.patch_ws)

    G.test_close()  # 12/19/2017 added by BW
    fc8_predictions = np.concatenate(fc8_predictions)
    patch_Hs = np.concatenate(patch_Hs)
    patch_Ws = np.concatenate(patch_Ws)

    # print patch_Hs
    # print patch_Ws

    r = np.sort(fc8_predictions, axis=None)
    r_no_keep = r[-nbest]
    print('Time taken: {}s'.format(time.time() - st_time))

    for pindex in range(fc8_predictions.shape[0]):
        for tindex in range(fc8_predictions.shape[1]):
            if fc8_predictions[pindex, tindex] < r_no_keep:
                continue
            else:
                I = drawRectangle(I, patch_Hs[pindex],
                                  patch_Ws[pindex], tindex, gsize)
                bestH = patch_Hs[pindex]
                bestW = patch_Ws[pindex]
                bestGraspAngle = tindex * (np.pi / 18.0) - np.pi / 2.0

    print('displaying image')
    cv2.imshow('image', I)
    cv2.waitKey(1)

    return bestH, bestW, bestGraspAngle
    # G.test_close()
