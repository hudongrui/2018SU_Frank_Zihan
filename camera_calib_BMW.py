import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo


def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    # /home/team18/.ros/camera_info/head_camera.yaml
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg


def getCamMatrx(cam_info_path = '/home/team18/.ros/camera_info/head_camera.yaml'):
    camInfo = yaml_to_CameraInfo(cam_info_path)
    cam_Matrx = np.reshape(camInfo.K, [3, 3])
    return cam_Matrx


# print(getCamMatrx())
# camInfo = yaml_to_CameraInfo('/home/team18/.ros/camera_info/head_camera.yaml')
#
# print('image_width: ', camInfo.width)
# print('image_height: ', camInfo.height)
# print('camera_matrix: ', camInfo.K)
# print('distortion_coef: ', camInfo.D)
# print('rectification_matrix: ', camInfo.R)
# print('projection_matrix: ', camInfo.P)
# print('distortion_model: ', camInfo.distortion_model)
#
# cam_Matrx = np.reshape(camInfo.K, [3, 3])
# print(cam_Matrx)


