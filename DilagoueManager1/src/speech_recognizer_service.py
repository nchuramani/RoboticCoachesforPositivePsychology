#!/usr/bin/env python

import speech_recognition as sr
import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

def handle_speech_recognition(req):
    r = sr.Recognizer()
    with sr.Microphone() as source:
        r.adjust_for_ambient_noise(source)
        rospy.loginfo('Say something!')
        audio = r.listen(source)

    try:
        # Using the default API key
        return tts_srvResponse(r.recognize_google(audio))
    except:
        return tts_srvResponse('Couldn\'t understand')

def speech_recognition_server():
    rospy.init_node('speech_recognizer_srv')

    s = rospy.Service('speech_recognizer_srv', tts_srv, handle_speech_recognition)
    rospy.loginfo('Speech recognition service is ready.')

    rospy.spin()

if __name__ == '__main__':
    speech_recognition_server()