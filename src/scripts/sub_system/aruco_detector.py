#!/usr/bin/env python3
import sys
import rospy
import cv2
import cv2.aruco as aruco
import time
import numpy as np
import math
from std_msgs.msg import UInt8
from kobot.msg import landmark_pose
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError


class arucoDetector:

    def __init__(self):
        # debug image for aruco detection
        self.image_pub = rospy.Publisher("detected_aruco",Image, queue_size=1)
        # handle tf in another node since tf and python3 have problems
        self.rt_pub = rospy.Publisher("landmark_pose",landmark_pose, queue_size=1)
        # subscribe to image to detect aruco
        self.image_sub = rospy.Subscriber("raspicam_node/image",Image,self.image_callback)
        self.bridge = CvBridge()
        # set default params 
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
        self.aruco_params.cornerRefinementMaxIterations = 100
        self.marker_len = 0.05 # m
        self.is_debug = True
        # load camera calibration
        if rospy.has_param('camera_calibration'):
            camera_calibration = rospy.get_param('camera_calibration')
            self.camera_matrix, self.dist_coeffs =\
                camera_calibration['camera_matrix'], camera_calibration['dist_coeffs'][0]
            self.camera_matrix = np.array(self.camera_matrix)
            self.dist_coeffs = np.array(self.dist_coeffs)
            rospy.loginfo("Using calibrated camera")
            self.camera_calibrated = True
        else:
            rospy.loginfo("Camera not calibrated")
            self.camera_calibrated = False
        # load additional params.
        if rospy.has_param('lba_params/d_detect'):
            self.detection_dist_thresh = rospy.get_param('lba_params/d_detect')
        else:
            self.detection_dist_thresh = 10 # m

    def pub_rt(self, ids, rvec, tvec):
        """
        Publish a msg. containing detected aruco id, rvec and tvec
        """
        rt_msg = rt()
        rt_msg.id = int(ids)
        rt_msg.rvec = rvec
        rt_msg.tvec = tvec
        self.rt_pub.publish(rt_msg)

    def image_callback(self,data):
        """
        Callback for the raw image from the camera
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # start_time = time.time()
            self.get_aruco(cv_image)
            # rospy.loginfo("Time Diff : {}".format(time.time()-start_time))
        except CvBridgeError as e:
            rospy.logerr(e)

    def get_aruco(self, frame):
        """
        Detect the aruco marker from the image 
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # crop image in vertical axis for min. computation
        h, w = gray.shape
        cropped_h = int(h / 6)
        x = 0
        # y = int((h - cropped_h) / 2)
        y = 0
        gray = gray[y:y+h, x:x+w]

        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)


        gray = aruco.drawDetectedMarkers(
            gray, corners, ids)
        if self.camera_calibrated and ids is not None:
            # aruco detected
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_len, self.camera_matrix, self.dist_coeffs)
            # in rads
            rvec = rvec[0][0]
            # in meters
            tvec = tvec[0][0]

            # calculate euclidian distance to marker
            d = float(np.linalg.norm(tvec))
            # rospy.loginfo("x : {}, y: {}, z: {}".format(tvec[0], tvec[1], tvec[2]))
            if d < self.detection_dist_thresh:
                self.pub_rt(ids[0], rvec, tvec)
            else:
                # distance is too large
                return
            if self.is_debug:
                # publish debug image with axes drawn on aruco
                self.debug_aruco_detection(gray,rvec, tvec, d)

    def debug_aruco_detection(self, gray, rvec, tvec, dist):
        """
        Debug aruco detection by drawing axes of Aruco Marker and distance
        to the image than publishing it
        """
        gray = aruco.drawAxis(
            gray, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_len)
        # write euclidian distance to marker center on image
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (320,160)
        fontScale              = 1
        fontColor              = (0,0,255)
        lineType               = 2
        cv2.putText(gray,str(dist), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)

        gray = self.bridge.cv2_to_imgmsg(gray, "mono8")
        self.image_pub.publish(gray)

def start(args):
    rospy.init_node('aruco_detector', anonymous=True)
    detector = arucoDetector()
    rospy.spin()
        

if __name__ == '__main__':
        start(sys.argv)

