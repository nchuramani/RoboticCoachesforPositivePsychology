#!/usr/bin/env python
import numpy as np
import cv2
from naoqi import ALProxy

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

PEPPER_ID = '127.0.0.1'
PEPPER_PORT = 9559

bridge = CvBridge()

# get NAOqi module proxy
videoDevice = ALProxy('ALVideoDevice', PEPPER_ID, PEPPER_PORT)

# subscribe top camera
AL_kTopCamera = 0
AL_kVGA = 2            # 640x480
AL_kBGRColorSpace = 13
captureDevice = videoDevice.subscribeCamera(
    "test", AL_kTopCamera, AL_kVGA, AL_kBGRColorSpace, 10)

# create image
width = 640
height = 480
image = np.zeros((height, width, 3), np.uint8)

def main():
    rospy.init_node('camera_publisher', anonymous=True)

    img_pub = rospy.Publisher('camera_channel', Image, queue_size=10)

    fps = 30
    rate = rospy.Rate(fps)

    frame_id = 0
    
    while not rospy.is_shutdown():

        # get image
        result = videoDevice.getImageRemote(captureDevice);

        if result == None:
            print 'cannot capture.'
        elif result[6] == None:
            print 'no image data string.'
        else:

            # translate value to mat
            values = map(ord, list(result[6]))
            i = 0
            for y in range(0, height):
                for x in range(0, width):
                    image.itemset((y, x, 0), values[i + 0])
                    image.itemset((y, x, 1), values[i + 1])
                    image.itemset((y, x, 2), values[i + 2])
                    i += 3

            try:
                img_msg = bridge.cv2_to_imgmsg(image)
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
        cv2.destroyAllWindows()