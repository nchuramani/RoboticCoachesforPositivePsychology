#!/usr/bin/env python

import speech_recognition as sr

from log_manager import LogManager

import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

# creating/connecting the main log file
lm = LogManager(rospy.get_param('logger'))

# creating speech recognition recognizer
recognizer = sr.Recognizer()

# sets microphone
mic = sr.Microphone()

# gets the user name
name = rospy.get_param('name')

# getting the values from the config file. they can be modified.
with open('./config.txt', 'r') as f:
    fLines = f.readlines()

    PAUSE_THRESHOLD = float(fLines[4].split('=')[1].strip())

def handle_speech_recognition(req):
    # if timeout value is not specified
    if req.inp == '':
        TIMEOUT = None
        lm.write('\nListening for speech...')
    else:
        TIMEOUT = int(req.inp)
        lm.write('\nListening for speech with ' + str(TIMEOUT) + ' sec timeout...')

    try:
        # listens to the user
        with mic as source:
            recognizer.dynamic_energy_threshold = False
            recognizer.pause_threshold = PAUSE_THRESHOLD
            audio = recognizer.listen(source, timeout=TIMEOUT)

        # converts the audio to text
        # uses the default API key (it can be changed)
        resp = recognizer.recognize_google(audio)
        lm.write(name.upper() + ": " + str(resp).capitalize())

        return tts_srvResponse(str(resp))

    except sr.WaitTimeoutError:
        lm.write('\nNot responded in ' + str(TIMEOUT) + ' seconds!')
        
        # if the user does not answer in the specified timeout time
        return tts_srvResponse('<NOT_RESPONDED>')

    except:
        lm.write('WARNING: Couldn\'t understand')

        # if the user cannot be understood by the speech recognition service
        return tts_srvResponse('<Couldn\'t understand!>')

def speech_recognition_server():
    # initializing the ROS node
    rospy.init_node('speech_recognizer_srv')
    lm.write('speech_recognizer_service.py node initialized.')

    # initializing the ROS service
    s = rospy.Service('speech_recognizer_srv', tts_srv, handle_speech_recognition)
    lm.write('Speech recognition service is ready.\n')

    rospy.spin()

if __name__ == '__main__':
    speech_recognition_server()