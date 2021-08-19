#!/usr/bin/env python

from gtts import gTTS
import os
import playsound

from log_manager import LogManager

import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

import urllib3

# preventing the gtts to write 'InsecureRequestWarning' error message to the terminal
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

# changing directory to the file's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# creating/connecting the main log file
lm = LogManager(str(rospy.get_param('logger')))

def handle_text_to_speech(req):
    # initializing the gtts
    tts = gTTS(text=req.inp, lang='en', slow=False)

    # sets the audio file's name
    sound = 'sound.mp3'
    tts.save(sound)

    lm.write('PEPPER: ' + req.inp)
    
    # plays the audio file
    playsound.playsound(sound, True)

    return tts_srvResponse('')

def text_to_speech_server():
    # initializing the ROS node
    rospy.init_node('text_to_speech_srv')
    lm.write('text_to_speech_service.py node initialized.')
    
    # initializing the ROS service
    s = rospy.Service("text_to_speech_srv", tts_srv, handle_text_to_speech)
    lm.write('Text to speech service is ready.')
    lm.write('Text to speech sound file will be saved to ' + os.path.join(os.getcwd(), 'sound.m3\n'))

    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()
