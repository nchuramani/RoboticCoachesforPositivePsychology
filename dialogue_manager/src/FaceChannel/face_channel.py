#!/usr/bin/env python

from Utils import imageProcessingUtil, modelDictionary, modelLoader
import numpy
import tensorflow as tf

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import keras.backend.tensorflow_backend as tb

import cv2

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

saveFrames = rospy.get_param('save_frames')

if saveFrames:
    # changing directory to the log files' directory to save frames
    os.chdir(lm.getFileDir())

    # creating frames folder if not
    if not os.path.isdir('frames'):
        os.mkdir('frames')

     # creating frames folder if not
    if not os.path.isdir('faces'):
        os.mkdir('faces')

    frameDir = os.path.join(os.getcwd(), 'frames')
    faceDir = os.path.join(os.getcwd(), 'faces')

# creating log file to keep the frames' dimensional outputs
lm_arousal_valence = LogManager('arousal_valence')
lm_arousal_valence.write('ORDER: [Arousal, Valence]')
lm_arousal_valence.separate(2)

# initializing the ROS node
rospy.init_node('face_channel', anonymous=True)
lm.write('face_channel.py node initialized.')

# creating publisher to publish face frames
face_frame_pub = rospy.Publisher('face_frames_channel', Image, queue_size=10)
lm.write('Face frames publisher is ready - [face_frames_channel] topic')

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
        except CvBridgeError as e:
            lm.write(e.message)
        
        # detects faces
        facePoints, face = imageProcessing.detectFace(frame)

        # uncomment below to see the frames during the experiment
        '''
        cv2.imshow('frames', frame)

        if cv2.waitKey(33) == 27:
            return
        '''

         # if a face is detected
        if not len(face) == 0:
            # pre-process the face
            processedFace = imageProcessing.preProcess(face, faceSize)
            
            # obtain dimensional classification
            dimensionalRecognition = numpy.array(modelDimensional.classify(processedFace))
            lm_arousal_valence.write(str(dimensionalRecognition).replace('\n', ''), printText=False)

            face = cv2.resize(face, (96, 96), interpolation = cv2.INTER_AREA)
            
            try:
                # converting image to publishable format
                img_msg = bridge.cv2_to_imgmsg(face)

                # publishes image data
                face_frame_pub.publish(img_msg)
        
            except CvBridgeError as e:
                lm.write(e.message)


            if saveFrames and str(rospy.get_param('current_state')) != "NONE":
                # creating the frame folder for current interaction state, if not exists
                if str(rospy.get_param('current_state')) not in os.listdir(frameDir):
                    os.mkdir(os.path.join(frameDir, str(rospy.get_param('current_state'))))
                    os.mkdir(os.path.join(faceDir, str(rospy.get_param('current_state'))))

                # saving frames to the specified folder for later analysis
                cv2.imwrite(os.path.join(frameDir, str(rospy.get_param('current_state')), 'frame_' + str(datetime.now()).replace(' ', '::') + '.jpg'), frame)

                # saving face frames to the specified folder for later analysis
                cv2.imwrite(os.path.join(faceDir, str(rospy.get_param('current_state')), 'face_frame_' + str(datetime.now()).replace(' ', '::') + '.jpg'), face)

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