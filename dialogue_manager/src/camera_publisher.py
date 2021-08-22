#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

from log_manager import LogManager

import numpy as np
import cv2

import os
from datetime import datetime

# creating/connecting the main log file
lm = LogManager('main')

saveFrames = rospy.get_param('save_frames')

if saveFrames:
    # changing directory to the log files' directory to save frames
    os.chdir(lm.getFileDir())

    # creating frames folder if not
    if not os.path.isdir('frames'):
        os.mkdir('frames')

    frameDir = os.path.join(os.getcwd(), 'frames')

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

        if saveFrames and str(rospy.get_param('current_state')) != "NONE":
            # creating the frame folder for current interaction state, if not exists
            if str(rospy.get_param('current_state')) not in os.listdir(frameDir):
                os.mkdir(os.path.join(frameDir, str(rospy.get_param('current_state'))))

            # saving frames to the specified folder for later analysis
            cv2.imwrite(os.path.join(frameDir, str(rospy.get_param('current_state')), 'frame_' + str(datetime.now()).replace(' ', '::') + '.jpg'), frame)

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