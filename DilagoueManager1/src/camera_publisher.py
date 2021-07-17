#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import cv2

bridge = CvBridge()

cap = cv2.VideoCapture(0)

def main():
    rospy.init_node('camera_publisher', anonymous=True)

    img_pub = rospy.Publisher('camera_channel', Image, queue_size=10)

    fps = cap.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)

    frame_id = 0

    while not rospy.is_shutdown() and cap.isOpened():
        rval, frame = cap.read()

        if not rval:
            rospy.loginfo('Could not grab camera frame.')
            break

        try:
            img_msg = bridge.cv2_to_imgmsg(frame)
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = str(frame_id)
            img_pub.publish(img_msg)
        
        except CvBridgeError as err:
            rospy.loginfo(err)

        frame_id += 1

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()