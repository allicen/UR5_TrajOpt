#!/usr/bin/env python3

import sys
import time
from math import sin, cos, sqrt, atan
import numpy as np
np.set_printoptions(threshold=sys.maxsize)

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import cv2
import io
import base64
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from ur5_husky_camera.msg import ImageCamera

import matplotlib.pyplot as plt

class Camera():

    def __init__(self):
        rospy.init_node('camera_depth', anonymous=True)

        self.rosbag = rospy.get_param("~rosbag")
        self.delay = rospy.get_param("~delay")
        gripper_h = rospy.get_param("~gripper_h")
        gripper_close_w = rospy.get_param("~gripper_close_w")
        self.add_size_contour = rospy.get_param("~add_size_contour")

        self.cv_bridge = CvBridge()

        self.ImageGripperDepth = None
        self.can_get_image_gripper_depth = False

        self.width = 1280
        self.height = 720

        self.gripper_busy = False

        self.line_height = 5

        self.image_border_start = (0, self.height - int(self.line_height/2))
        self.image_border_end = (self.width - int(self.line_height/2), self.height - int(self.line_height/2))

        self.gripper_border_start = (0, self.height - int(self.line_height/2) - gripper_h)
        self.gripper_border_end = (self.width - int(self.line_height/2), self.height - int(self.line_height/2) - gripper_h)

        self.gripper_close_start = (int(self.width/2 - self.line_height/2) + gripper_close_w, 0)
        self.gripper_close_end = (int(self.width/2 - self.line_height/2) + gripper_close_w, self.height)
        

        # Images from RealSense 
        rospy.Subscriber("/pub/realsense_gripper/color/image_raw", ImageCamera, self.camera_gripper)
        rospy.Subscriber("/realsense_gripper/color/image_raw/compressed", CompressedImage, self.camera_gripper)

        rospy.Subscriber("/zed_node/right_raw/image_raw_color/compressed", ImageCamera, self.camera_robot)
        rospy.Subscriber("/realsense_gripper/aligned_depth_to_color/image_raw", Image, self.camera_gripper_depth)
        rospy.Subscriber("/pub/realsense_gripper/aligned_depth_to_color/image_raw", Image, self.camera_gripper_depth)

        rospy.Subscriber("/rviz1/camera1/image", Image, self.camera_rviz)
        self.pub_rviz = rospy.Publisher('rviz_camera', ImageCamera, queue_size=10)

        self.pub_gripper = rospy.Publisher('gripper_camera', ImageCamera, queue_size=10)
        self.pub_gripper_depth = rospy.Publisher('gripper_camera_depth', ImageCamera, queue_size=10)
        self.pub_robot = rospy.Publisher('robot_camera', ImageCamera, queue_size=10)
        self.pub_gripper_state = rospy.Publisher('gripper_state_camera_depth', String, queue_size=10)
        self.pub_gripper_depth_small = rospy.Publisher('gripper_camera_depth_image', Image, queue_size=10)
        self.rate = rospy.Rate(30)

        rospy.on_shutdown(self.shutdown)

    def camera_rviz(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.pub_rviz.publish(self.createMessage(cv_image))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))


    def base64decode(self, image):
        decoded_data = base64.b64decode(image)
        np_data = np.fromstring(decoded_data,np.uint8)
        img = cv2.imdecode(np_data,cv2.IMREAD_UNCHANGED)
        return img


    def createMessage(self, image):
        img = self.cv_bridge.cv2_to_imgmsg(image)
        _, buffer_img = cv2.imencode('.jpg', image)

        msg_img = ImageCamera()
        msg_img.data = base64.b64encode(buffer_img).decode("utf-8")
        msg_img.encoding = 'base64'
        msg_img.width = img.width
        msg_img.height = img.height
        return msg_img


    def camera_gripper(self, msg):
        if self.rosbag:
            try:
                cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                self.pub_gripper.publish(self.createMessage(cv_image))
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            self.pub_gripper.publish(msg)


    def camera_robot(self, msg):
        if self.rosbag:
            pass
        else:
            self.pub_robot.publish(msg)


    def camera_gripper_depth(self, msg):
        # delay from get image camera
        if self.can_get_image_gripper_depth:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
                self.ImageGripperDepth = cv_image
                self.can_get_image_gripper_depth = False

            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))
        else:
            self.ImageGripperDepth = None


    def process_depth_image(self, cv_image):
        depth_array = np.array(cv_image, dtype=np.float32)

        bl = cv2.medianBlur(depth_array, 5)

        hsv_min = np.array((0, 0, 0), np.uint8)
        hsv_max = np.array((0, 0, 0), np.uint8)
        
        img = cv2.cvtColor(bl, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        dilated = cv2.dilate(thresh, kernel)
        
        # cnts, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # for c in cnts:
        #     x,y,w,h = cv2.boundingRect(c)
        #     ROI = depth_array[y:y+h, x:x+w]
        #     mean, STD  = cv2.meanStdDev(ROI)

        #     offset = 0.2
        #     clipped = np.clip(depth_array, mean - offset*STD, mean + offset*STD).astype(np.uint8)
        #     depth_array = cv2.normalize(clipped, clipped, 0, 255, norm_type=cv2.NORM_MINMAX)


        
        # return depth_array


        cv2.normalize(bl, bl, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
        # return depth_array

        colormap = plt.get_cmap('inferno')
        heatmap = (colormap(bl) * 2**16).astype(np.uint16)[:,:,:3]
        heatmap = cv2.cvtColor(heatmap, cv2.COLOR_RGB2BGR)

        return heatmap
       
        bl = cv2.medianBlur(depth_array, 5)

        hsv_min = np.array((0, 0, 0), np.uint8)
        hsv_max = np.array((0, 0, 0), np.uint8)
        
        img = cv2.cvtColor(bl, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        thresh = cv2.inRange(hsv, hsv_min, hsv_max)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        dilated = cv2.dilate(thresh, kernel)

        contours_prev, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find broken contours
        for cnt in contours_prev:
            left_point, right_point = 10000, -1
            for ci in cnt:
                if ci[0][1] == self.height-1:
                    if ci[0][0] < left_point:
                        left_point = ci[0][0]
                    if ci[0][0] > right_point:
                        right_point = ci[0][0]
            
            if left_point != 10000 and right_point != -1:
                # fix broken contours
                cv2.line(img, (left_point-1, self.image_border_start[1]), (right_point+1, self.image_border_start[1]), (0,0,0), 10)

        # re-find contours
        thresh = cv2.inRange(img, hsv_min, hsv_max)
        dilated = cv2.dilate(thresh, kernel)
        contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cntrs_find = []
        cntrs_white_masked = []

        # apply masks to unnecessary contours
        for cnt in contours:
            if len(cnt) > 1:
                add_mask = True
                for ci in cnt:
                    if ci[0][1] > self.gripper_border_start[1]:
                        for cj in cnt:
                            if cj[0][1] >= self.image_border_start[1]:
                                cntrs_find.append(cnt)
                                add_mask = False
                                break
                if add_mask:
                    cntrs_white_masked.append(cnt)

        # add masks            
        img = cv2.fillPoly(img, contours, [0,0,0])            
        img = cv2.fillPoly(img, cntrs_white_masked, [255,255,255])

        cntrs_find = sorted(cntrs_find, key=cv2.contourArea, reverse=True)

        if len(cntrs_find) == 0:
            return img

        long_contour = cntrs_find[0]

        # thicken the longest contour
        if self.add_size_contour > 0:
            for i in range(len(long_contour)):
                if i == 0:
                    continue
                c = long_contour[i][0]
                c_prev = long_contour[i-1][0]
                cv2.line(img, (c[0], c[1]), (c_prev[0], c_prev[1]), (0,0,0), self.add_size_contour*2)

            # re-find contours
            thresh = cv2.inRange(img, hsv_min, hsv_max)
            dilated = cv2.dilate(thresh, kernel)
            contours, _ = cv2.findContours(dilated.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                long_contour =  sorted(contours, key=cv2.contourArea, reverse=True)[0]

        cv2.drawContours(img, long_contour, -1, (0,0,255), 6)

        # draw lines through points
        L1 = self.line(self.image_border_start, self.image_border_end)
        L2 = self.line(self.gripper_border_start, self.gripper_border_end)
        L3 = self.line(self.gripper_close_start, self.gripper_close_end)

        # find the intersection points of straight lines
        R1 = self.intersection(L1, L3)
        R2 = self.intersection(L2, L3)

        msg = String()
        msg.data = 'ready'

        if R1 and R2:
            # check if the point is inside the contour
            dist1 = cv2.pointPolygonTest(long_contour, R1, True)
            dist2 = cv2.pointPolygonTest(long_contour, R2, True)

            # if there are both points inside the contour, then the gripper is busy
            if dist1 > 0 and dist2 > 0:
                self.gripper_busy = True
                msg.data = 'busy'
            else:
                msg.data = 'ready'
                self.gripper_busy = False
        else:
            msg.data = 'ready'
            self.gripper_busy = False

        # send result
        self.pub_gripper_state.publish(msg)

        return img


    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C

    def intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False


    def spin(self):
        time_now = rospy.Time.now()
        time_prev = time_now

        while not rospy.is_shutdown():

            # set delay for get image from camera
            now = rospy.Time.now()
            delta_time = now - time_prev

            if delta_time.secs >= self.delay:
                self.can_get_image_gripper_depth = True
                time_prev = now

            if self.ImageGripperDepth is not None:
                img = self.process_depth_image(self.ImageGripperDepth)

                # add info
                # cv2.line(img, self.gripper_border_start, self.gripper_border_end, (255,0,0), self.line_height)
                # cv2.line(img, self.gripper_close_start, self.gripper_close_end, (255,0,0), self.line_height)

                # Text style
                font = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (10, self.gripper_border_start[1]-10)
                fontScale = 2
                fontColor = (255,0,0)
                thickness = 3
                lineType = 5

                # send to Image
                img_sent_rviz = img.copy()
                # cv2.putText(img_sent_rviz,'Obj in gripper: ', (50, 150), font, 4, (255,0,0), 12, lineType)
                # if self.gripper_busy:
                #     cv2.putText(img_sent_rviz,'YES', (1000, 150), font, 4, (0,255,0), 12, lineType)
                # else:
                #     cv2.putText(img_sent_rviz,'NO', (1000, 150), font, 4, (0,0,255), 12, lineType)

                # Img sent to rviz
                # img_sent_rviz = img_sent_rviz.astype(np.uint8)
                # cv2.convertScaleAbs(img_sent_rviz, img_sent_rviz, 255, 0)
                output = cv2.resize(img_sent_rviz, (800, int(self.height * 800 / self.width)))
                self.pub_gripper_depth_small.publish(self.cv_bridge.cv2_to_imgmsg(output))

                # add text
                # cv2.putText(img,'Gripper Line', bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

                # send image to GUI
                msg_pub = self.createMessage(img)
                self.pub_gripper_depth.publish(msg_pub)
                
                cv2.imshow("depth", output)

            cv2.waitKey(3)
            self.rate.sleep()

    def shutdown(self):
        rospy.sleep(1)


camera = Camera()
camera.spin()
