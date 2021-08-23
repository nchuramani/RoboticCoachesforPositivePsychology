#!/usr/bin/env python
import numpy as np
import cv2
from naoqi import ALProxy

from log_manager import LogManager

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import os
from datetime import datetime

# changing directory to the file's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# creating/connecting the main log file
lm = LogManager('main')

saveFrames = rospy.get_param('save_frames')

# getting the values from the config file. they can be modified.
with open('./config.txt', 'r') as f:
    fLines = f.readlines()

    PEPPER_IP = fLines[0].split('=')[1].strip()
    PEPPER_PORT = int(fLines[1].split('=')[1].strip())
    PEPPER_CAMERA_INDEX = int(fLines[5].split('=')[1].strip())
    PEPPER_CAMERA_RESOLUTION = int(fLines[6].split('=')[1].strip())
    PEPPER_CAMERA_COLORSPACE = int(fLines[7].split('=')[1].strip())
    PEPPER_CAMERA_FPS = int(fLines[8].split('=')[1].strip())

# creating bridge to convert images to ROS publishable format
bridge = CvBridge()

# gets NAOqi module proxy
videoDevice = ALProxy('ALVideoDevice', PEPPER_IP, PEPPER_PORT)
lm.write('Subscribed to Pepper\'s camera')

# subscribes to pepper's camera
AL_kTopCamera = PEPPER_CAMERA_INDEX
AL_kVGA = PEPPER_CAMERA_RESOLUTION  # 640x480 for AL_kVGA = 2 (recommended)
AL_kBGRColorSpace = PEPPER_CAMERA_COLORSPACE
fps = PEPPER_CAMERA_FPS

captureDevice = videoDevice.subscribeCamera(
    "test", AL_kTopCamera, AL_kVGA, AL_kBGRColorSpace, 10)

#other options for captureDevice can be found at the official website of SoftBank.

lm.separate()
lm.write('\nPEPPER CAMERA OPTIONS', False)
lm.separate(1)
lm.write('AL_kTopCamera: ' + str(AL_kTopCamera) +
        '\nAL_kVGA: ' + str(AL_kVGA) + 
        '\nAL_kBGRColorSpace: ' + str(AL_kBGRColorSpace) +
        '\nFPS: ' + str(fps))

# create image
width = 640
height = 480
image = np.zeros((height, width, 3), np.uint8)

lm.write('\nFRAME OPTIONS', False)
lm.separate(1)
lm.write('Width: ' + str(width) +
        '\nHeight: ' + str(height))
lm.separate()

def main():
    # initializing the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    lm.write('pepper_camera_publisher.py node initialized.')

    # creating publisher to publish images
    img_pub = rospy.Publisher('camera_channel', Image, queue_size=10)
    lm.write('Pepper camera publisher is ready [camera_channel topic]')

    # keeps fps value to wait for the camera
    rate = rospy.Rate(fps)

    # frame counter
    frame_id = 0
    
    while not rospy.is_shutdown():

        # gets image
        result = videoDevice.getImageRemote(captureDevice)

        if result == None:
            lm.write('WARNING: Camera cannot capture frames!')
            print 'cannot capture.'
        elif result[6] == None:
            lm.write('WARNING: No mat information for frame!')
            print 'no image data string.'
        else:

            # translates values to matrices
            values = map(ord, list(result[6]))
            i = 0
            for y in range(0, height):
                for x in range(0, width):
                    image.itemset((y, x, 0), values[i + 0])
                    image.itemset((y, x, 1), values[i + 1])
                    image.itemset((y, x, 2), values[i + 2])
                    i += 3

            try:
                # converting image to publishable format
                img_msg = bridge.cv2_to_imgmsg(image)

                # sets frame id
                img_msg.header.frame_id = str(frame_id)

                # publishes image data
                img_pub.publish(img_msg)

            except CvBridgeError as e:
                lm.write(e.message)

            frame_id += 1

            # to wait camera
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()