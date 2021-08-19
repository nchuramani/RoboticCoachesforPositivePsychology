#!/usr/bin/env python

from Utils import imageProcessingUtil, modelDictionary, modelLoader
import numpy
import tensorflow as tf

import rospy

from sensor_msgs.msg import Image
from dialogue_manager.msg import Emotion

from cv_bridge import CvBridge, CvBridgeError

import keras.backend.tensorflow_backend as tb

#import cv2

import os
import sys

# adding parent directory of the file to the system to import log manager
parentDir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(parentDir)

from log_manager import LogManager

# changing directory to the file's directory
os.chdir(os.path.dirname(__file__))

# creating/connecting the main log file
lm = LogManager(str(rospy.get_param('logger')))

# creating log file to keep the frames' dimensional outputs
lm_frame = LogManager(str(rospy.get_param('logger')) + '_frames')
lm_frame.write('ORDER: [Arousal, Valence]')

# initializing the ROS node
rospy.init_node('face_channel', anonymous=True)
lm.write('face_channel.py node initialized.')

# creating publisher to publish frames' dimensional outpus
category_pub = rospy.Publisher('emotion_channel', Emotion, queue_size=10)
lm.write('Face Channel publisher is ready - [emotion_channel] topic')

# creating bridge to convert ROS image messages to processable frames
bridge = CvBridge()

config = tf.compat.v1.ConfigProto()
config.gpu_options.allow_growth=True
sess = tf.compat.v1.Session(config=config)

# input size for the dimensional model
faceSize = (64,64)

frameSize = (480, 640)

lm.separate()
lm.write('FACE CHANNEL FRAME OPTIONS', False)
lm.separate(1)
lm.write('Frame size: ' + str(frameSize[0]) + 'x' + str(frameSize[1]) + '\n' +
        'Face size: ' + str(faceSize[0]) + 'x' + str(faceSize[1]))

lm.separate()

# loading dimensional model
modelDimensional = modelLoader.modelLoader(modelDictionary.DimensionalModel)
lm.write('Dimensional model for Face Channel is loaded')

imageProcessing = imageProcessingUtil.imageProcessingUtil()

def callback(data):
        #to prevent "'thread._local' object has no attribute 'value'" error
        tb._SYMBOLIC_SCOPE.value = True

        try:
            frame = bridge.imgmsg_to_cv2(data)
        except CvBridgeError as err:
            rospy.loginfo(err)
        
        # detects faces
        facePoints, face = imageProcessing.detectFace(frame)

        '''
        cv2.imshow('pepper', frame)

        if cv2.waitKey(33) == 27:
            return
        '''

         # if a face is detected
        if not len(face) == 0:
            # pre-process the face
            face = imageProcessing.preProcess(face, faceSize)
            
            # obtain dimensional classification
            dimensionalRecognition = numpy.array(modelDimensional.classify(face))
            lm_frame.write(str(dimensionalRecognition).replace('\n', ''), printText=False)

            # creating ROS message
            msg = [dimensionalRecognition[0],
                   dimensionalRecognition[1]]

            # publishing ROS message
            category_pub.publish(msg)


def listener():
    rospy.Subscriber('camera_channel', Image, callback)
    lm.write('Face Channel subscriber is ready - [camera_channel] topic')

    rospy.spin()

if __name__ == '__main__':
    listener()
    lm_frame.close()
