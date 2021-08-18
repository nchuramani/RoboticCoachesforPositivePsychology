#!/usr/bin/env python

from naoqi import ALProxy

from log_manager import LogManager

import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

import os

# changing directory to the file's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# creating/connecting the main log file
lm = LogManager(str(rospy.get_param('logger')))

# getting the values from the config file. they can be modified.
with open('./config.txt', 'r') as f:
    fLines = f.readlines()

    PEPPER_IP = fLines[0].split('=')[1].strip()
    PEPPER_PORT = int(fLines[1].split('=')[1].strip())
    PEPPER_SPEAK_SPEED = int(fLines[10].split('=')[1].strip())

# gets NAOqi module proxies for text to speech and running behaviors 
tts = ALProxy("ALTextToSpeech", PEPPER_IP, PEPPER_PORT)
tts.setParameter("speed", PEPPER_SPEAK_SPEED)

behavior = ALProxy('ALBehaviorManager', PEPPER_IP, PEPPER_PORT)

def handle_text_to_speech(req):
    lm.write('PEPPER: ' + req.inp)

    # makes the robot speak
    tts.say(req.inp)
    
    # if no behavior is specified
    if req.behavior != '':
        # gets behavior path
        behaviorPath = '.lastUploadedChoregrapheBehavior/' + req.behavior
        
        # makes the robot perform the behavior
        behavior.runBehavior(behaviorPath)
        lm.write('Running Pepper behavior: ' + behaviorPath)

    #return a blank string because the output is not needed
    return tts_srvResponse('')

def text_to_speech_server():
    # initializing the ROS node
    rospy.init_node('text_to_speech_srv')
    lm.write('pepper_tts_service.py node initialized.')
    
    # initializing the ROS service
    s = rospy.Service("text_to_speech_srv", tts_srv, handle_text_to_speech)
    lm.write('Text to speech service is ready.')

    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()
