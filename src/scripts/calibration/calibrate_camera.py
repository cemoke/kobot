#!/usr/bin/env python3
import sys
import rospy
import cv2
import cv2.aruco as aruco
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os


class image_converter:
    """
    Subscribes to the image feed 
    and tries to detect charuco board
    if sucessful stores intrinsic camera params.
    in a file
    """

    def __init__(self):
        self.bridge = CvBridge()

        rospy.Subscriber("/raspicam_node/image",
            Image,
            self.img_callback,
            queue_size=1)

        self.image_pub = rospy.Publisher("/charuco/image",
            Image,
            queue_size=1)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        # create our charuco board used for calibration
        # length unit is in m
        self.board = aruco.CharucoBoard_create(10, 8, 0.03, 0.015, self.aruco_dict)

        self.corner_list = []
        self.id_list = []
        self.counter = 0

    def img_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.get_aruco(cv_image)
        except CvBridgeError as e:
            rospy.loginfo(e)

    def get_aruco(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.aruco_params)
        num_detected_aruco = 0
        # Get charuco corners and ids from detected aruco markers
        if ids is not None:
            rospy.loginfo("Num. of Detected Arucos : {}".format(len(ids)))
            response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=self.board)
            num_detected_aruco = response
        if num_detected_aruco > 20:
            img = aruco.drawDetectedCornersCharuco(
                image=gray,
                charucoCorners=charuco_corners,
                charucoIds=charuco_ids)
            self.img_shape = img.shape
            rospy.loginfo("Charuco is found")
            self.corner_list.append(charuco_corners)
            self.id_list.append(charuco_ids)
            self.counter += num_detected_aruco
            # draw the Charuco board we've detected to show our 
            # calibrator the board was properly detected
            img = self.bridge.cv2_to_imgmsg(img, "mono8")
            self.image_pub.publish(img)
        else:
            rospy.loginfo("Charuco is not found")
        if self.counter >=1200:
            self.calibrate()
        else:
            rospy.loginfo(self.counter)



    def calibrate(self):
        """
        Try to perform calibration by using the collected aruco corners
        """
        try:
            cal = cv2.aruco.calibrateCameraCharuco(
                self.corner_list,
                self.id_list,
                self.board,
                self.img_shape,
                None,
                None)
            # save camera matrix and distortion coeffs.
            self.save_calibration(cal[1], cal[2])
            rospy.loginfo(
                "Camera calibration is successful with" +
                "{} different Aruco markers".format(self.counter))

            rospy.signal_shutdown("calibrated")
        except Exception as e:
            rospy.loginfo("Exception raised on camera calibration")
            rospy.loginfo(e)

    def save_calibration(self, mtx, dist):
        mtx = mtx.tolist()
        dist = dist.tolist()
        rospy.loginfo("Camera Matrix : {}".format(mtx))
        rospy.loginfo("Dist Coeffs : {}".format(dist))
        rospy.set_param('camera_calibration',{'camera_matrix': mtx, 'dist_coeffs': dist})
        # dump params to the calibration file
        os.system('rosparam dump ~/kobot_ws/src/kobot/config/camera_calibration.yml')



def main(args):
    rospy.init_node('image_converter', anonymous=True)

    ic = image_converter()
    rospy.spin()
        

if __name__ == '__main__':
        main(sys.argv)

