#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from log_manager import LogManager

import numpy as np
import cv2

# creating/connecting the main log file
lm = LogManager(rospy.get_param('logger'))

# creating bridge to convert images to ROS publishable format
bridge = CvBridge()

# subscribes to computer camera
cap = cv2.VideoCapture(0)
lm.write('Subscribed to camera')

def main():
    # initializing the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    lm.write('camera_publisher.py node initialized.')

    # creating publisher to publish images
    img_pub = rospy.Publisher('camera_channel', Image, queue_size=10)
    lm.write('Pepper camera publisher is ready - [camera_channel] topic')

    # gets the fps of the camera
    fps = cap.get(cv2.CAP_PROP_FPS)
    lm.write('Camera FPS: ' + str(fps))

    # keeps fps value to wait for the camera
    rate = rospy.Rate(fps)

    # frame counter
    frame_id = 0

    while not rospy.is_shutdown() and cap.isOpened():
        # reads frames from camera
        rval, frame = cap.read()

        if not rval:
            lm.write('WARNING: Could not grab camera frame!')
            break

        try:
            # converting image to publishable format
            img_msg = bridge.cv2_to_imgmsg(frame)

            # sets frame id
            img_msg.header.frame_id = str(frame_id)

            # publishes image data
            img_pub.publish(img_msg)
        
        except CvBridgeError as err:
            rospy.loginfo(err)

        frame_id += 1

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    finally:
        # closes camera and openCV windows (if opened)
        cap.release()
        cv2.destroyAllWindows()