#!/usr/bin/env python3
from __future__ import print_function

# import roslib
# roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

import math

'''gets the x coordinate representing the centre of the road'''
def get_line_centre_x(grayFrame, y, lowerThres, upperThres):
    leftCoords = -1
    rightCoords = -1
    line = grayFrame[y] 
    # print(line)
    isInLine = False
    # first instance that line is detected (within threshold), leftmost pixel of road.
    # first instance from line --> off the line, rightmost pixel of road.
    for i in range(0,len(line)):
        if not isInLine and (line[i] >= lowerThres and line[i] <= upperThres):
            leftCoords = i
            isInLine = True
        elif isInLine and (line[i] < lowerThres or line[i] > upperThres):
            rightCoords = i
            break
    print(leftCoords, rightCoords)
    return (leftCoords + rightCoords) // 2

def get_pixels(image, fractional_row, fractional_col):
    rows, cols, = image.shape
    # print("r, c: ", rows, cols)
    row_pix = int((rows-1) * fractional_row)
    col_pix = int((cols-1) * fractional_col)
    # print("rp, cp:", row_pix, col_pix)
    return (row_pix, col_pix)

class image_converter:
    def __init__(self):
        self.twist_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        self.move = Twist()

        self.move.linear.x = 0.5
        self.prev_centre_x = 400
        
    def update_movement(self, centre_x, max_pixel):
        max_dist_offset = max_pixel / 2
        # pixels 0 to 800 so centre max diff = 400
        kp = 1.8 
        kd = 0
        # if centre_x < max_pixel / 2:
        #     self.move.angular.z = 0.5
        # else:
        #     self.move.angular.z = -0.5
        self.move.angular.z = kp*(max_dist_offset - centre_x)/(max_dist_offset) + kd*(self.prev_centre_x-centre_x)/(max_dist_offset)
        self.prev_centre_x = centre_x
        print("current angular z:", self.move.angular.z)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
        read_row,_ = get_pixels(gray, 0.7, 1)
        lowerThres = 40
        upperThres = 80
        centre = get_line_centre_x(gray, read_row, lowerThres, upperThres)
        self.update_movement(centre, len(gray[0]))
        # image
        # cv2.circle(cv_image, (read_row, centre), 10, 255)
        cv2.imshow("Image window", gray)
        cv2.waitKey(3)

        try:
            self.twist_pub.publish(self.move)
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)