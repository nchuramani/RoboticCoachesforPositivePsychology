#!/usr/bin/env python

import speech_recognition as sr

from log_manager import LogManager

import rospy
from dialogue_manager.srv import tts_srv, tts_srvResponse

import os

# changing directory to the file's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# getting the values from the config file. they can be modified.
with open('./config.txt', 'r') as f:
    fLines = f.readlines()

    MICROPHONE_NAME = fLines[9].split('=')[1].strip()

# creating/connecting the main log file
lm = LogManager('main')

# creating speech recognition recognizer
recognizer = sr.Recognizer()
print(sr.Microphone.list_microphone_names())

# sets microphone
micIndex = sr.Microphone.list_microphone_names().index(MICROPHONE_NAME)
mic = sr.Microphone(device_index=micIndex)
lm.write('Microphone index is chosen: ' + str(micIndex))
lm.write('Microphone name: ' + MICROPHONE_NAME)

# gets the user name
name = str(rospy.get_param('name'))

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
        return tts_srvResponse('<NOT_UNDERSTOOD>')

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