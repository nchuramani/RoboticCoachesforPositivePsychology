#!/usr/bin/env python

from Utils import imageProcessingUtil, modelDictionary, modelLoader
import numpy
import tensorflow as tf

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import keras.backend.tensorflow_backend as tb

#import cv2

import os
import sys

from datetime import datetime

# adding parent directory of the file to the system to import log manager
parentDir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(parentDir)

from log_manager import LogManager

# changing directory to the file's directory
os.chdir(os.path.dirname(__file__))

# creating/connecting the main log file
lm = LogManager('main')

# creating log file to keep the frames' dimensional outputs
lm_arousal_valence = LogManager('arousal_valence')
lm_arousal_valence.write('ORDER: [Arousal, Valence]')
lm_arousal_valence.separate(2)

# initializing the ROS node
rospy.init_node('face_channel', anonymous=True)
lm.write('face_channel.py node initialized.')

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
            lm_arousal_valence.write(str(dimensionalRecognition).replace('\n', ''), printText=False)

            # creates message to save the list
            text = str(datetime.now()).replace(' ', '_') + ' - ' + str(dimensionalRecognition).replace('\n', '')

            # adds value to the list that is held in a rosparam called 'arousal_valence'
            if rospy.get_param('arousal_valence') != 'NONE':
                rospy.set_param('arousal_valence', rospy.get_param('arousal_valence') + [text])

            else:
                rospy.set_param('arousal_valence', [text])

def listener():
    rospy.Subscriber('camera_channel', Image, callback)
    lm.write('Face Channel subscriber is ready - [camera_channel] topic')

    rospy.spin()

if __name__ == '__main__':
    listener()
    lm_arousal_valence.close()