#!/usr/bin/env python

from gtts import gTTS
import os
import playsound
import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

def handle_text_to_speech(req):
    tts = gTTS(text=req.inp, lang='en', slow=False)
    sound = 'sound.mp3'
    tts.save(sound)
    
    # Playing the converted file
    playsound.playsound(sound, True)

    return tts_srvResponse('')

def text_to_speech_server():
    rospy.init_node('text_to_speech_srv')
    
    s = rospy.Service("text_to_speech_srv", tts_srv, handle_text_to_speech)
    rospy.loginfo('Text to speech service is ready.')

    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()