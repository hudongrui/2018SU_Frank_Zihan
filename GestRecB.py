#!/usr/bin/python3

import json
import os
import time


class GestureRecognition:
    def __init__(self):
        # for debugging purposes
        self.openposeDir = "build/examples/openpose/openpose.bin"

    def execute(self, input_file_dir, output_file_dir,headDisplay):
        # must be in openpose to be able to run openpose
        os.chdir("/home/team18/openpose")
        os.system("streamer -f jpeg -o " + input_file_dir + "/webcam.jpeg -c /dev/video1")
        # time.sleep(0.5)
        os.system("streamer -f jpeg -o " + input_file_dir + "/webcam.jpeg -c /dev/video1")
        headDisplay.display_image("/home/team18/Grasp-Detector-master/sawyer_head/got_it.JPG")
        os.system(
            self.openposeDir + " --image_dir " + input_file_dir + " --write_keypoint_json " + output_file_dir +
            " --write_images " + output_file_dir + " --no_display")

        data = json.load(open(os.path.join(output_file_dir, "webcam_keypoints.json")))

        # only can handle seeing person0
        elbowR = data["people"][0]["pose_keypoints"][9]
        handR = data["people"][0]["pose_keypoints"][12]
        elbowL = data["people"][0]["pose_keypoints"][18]
        handL = data["people"][0]["pose_keypoints"][21]

        # direction will be positive if pointing right and negative if pointing left
        DirR = elbowR - handR
        DirL = elbowL - handL

        # Find which absolute distance is greater --> this will be the actual
        # direction that the user is pointing
        if abs(DirL) <= abs(DirR):
            # pointing right with respect to the user is given value of 0
            this_way = 0
        else:
            # pointing left with respect to the user is given value of 1, defaults to pointing left
            this_way = 1

        os.chdir("/home")
        return this_way