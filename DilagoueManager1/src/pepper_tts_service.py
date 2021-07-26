#!/usr/bin/env python

from naoqi import ALProxy
import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

PEPPER_IP = '127.0.0.1'
PEPPER_PORT = 9559

tts = ALProxy("ALTextToSpeech", PEPPER_IP, PEPPER_PORT)
behavior = ALProxy('ALBehaviorManager', PEPPER_IP, PEPPER_PORT)

def handle_text_to_speech(req):
    tts.say(req.inp)
    
    if req.behavior != '':
        behavior.runBehavior('.lastUploadedChoregrapheBehavior/' + req.behavior)

    return tts_srvResponse('')

def text_to_speech_server():
    rospy.init_node('text_to_speech_srv')
    
    s = rospy.Service("text_to_speech_srv", tts_srv, handle_text_to_speech)
    rospy.loginfo('Text to speech service is ready.')

    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()