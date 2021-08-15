#!/usr/bin/env python

import nlg

import time
import os
import random
import sys

import json
import numpy as np

import smach
import smach_ros

from log_manager import LogManager

import rospy
from dialogue_manager.srv import *
from dialogue_manager.msg import Emotion

import rosnode
import rosgraph

# changing directory to the file's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# getting the name of the user
name = rospy.get_param('name')

# getting the values from the config file. they can be modified.
with open('./config.txt', 'r') as f:
    fLines = f.readlines()

    TIMEOUT = fLines[2].split('=')[1].strip()
    DELAY = float(fLines[3].split('=')[1].strip())
    FACE_CHANNEL_SENSITIVITY = int(fLines[9].split('=')[1].strip())

# creating the main log file
lm = LogManager(rospy.get_param('logger'))

# creating log file for the state transitions and dialogue table
lm_flow = LogManager(rospy.get_param('logger') + '_flow')
lm_flow.createTable(['State', 'Condition', 'Speaker', 'Dialogue'])

# creating/connecting log file to get the frames' dimensional outputs
lm_frame = LogManager(rospy.get_param('logger') + '_frames')

# initializing the ROS node
rospy.init_node('state_manager', anonymous=True)
lm.write('state_manager.py node initialized.')

# arguments for the knowledge base
rospy.wait_for_service('speech_recognizer_srv')
rospy.wait_for_service('text_to_speech_srv')

try:
    # creating service proxy to use the text to speech service
    tts_ros_proxy = rospy.ServiceProxy('text_to_speech_srv', tts_srv)

    # creating service proxy to use the speech recognition service
    speech_ros_proxy = rospy.ServiceProxy('speech_recognizer_srv', tts_srv)

except rospy.ServiceException as e:
    lm.write('WARNING: Service call failed {}!'.format(e))

def tts_proxy(inp, behavior=''):
    tts_ros_proxy(inp, behavior)
    lm_flow.tableAddRow(['PEPPER', inp])

def speech_proxy(timeout=''):
    speech_text = speech_ros_proxy(timeout, '')
    lm_flow.tableAddRow([name.upper(), speech_text.outp])
    return speech_text

class PredProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                        outcomes=['introduction', 'goodbye', 
                                'past_impactful', 'past_grateful', 'past_accomplishments',
                                'present_impactful', 'present_grateful', 'present_accomplishments',
                                'future_impactful', 'future_grateful', 'future_accomplishments',
                                'feedback'],
                        input_keys=['kb_args', 'task', 'interaction'],
                        output_keys=['kb_args', 'status', 'condition'])

        self.first_time = True

    def execute(self, userdata):
        lm.write('\nExecuting state PREDPROCESS')
        lm_flow.setTableState('PREDPROCESS')

        if self.first_time:
            tts_proxy(nlg.dialogue['introduction'][0], 'hello')
            tts_proxy(nlg.dialogue['introduction'][1])

            lm.write('User name is initialized as [' + name.upper() + ']')

            userdata.kb_args['name'] = name

            self.first_time = False

            return 'introduction'

        else:
            #choosing the condition randomly
            condition = random.choice(['c1','c2','c3'])
            userdata.condition = condition
            lm_flow.setTableCondition(condition.upper())
            lm.write('Dialogue condition is randomly selected: [' + condition.upper() + ']')

            if userdata.interaction == 'feedback':
                return 'feedback'

            return userdata.task + '_' + userdata.interaction


class Introduction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'introduction'],
                             input_keys=['kb_args'],
                             output_keys=['status', 'kb_args'])

    def execute(self, userdata):
        lm.write('\nExecuting state INTRODUCTION')
        lm_flow.setTableState('INTRODUCTION')

        sentence = nlg.dialogue['introduction'][2].format(userdata.kb_args['name'])
        sentence += nlg.dialogue['introduction'][3]
        tts_proxy(sentence)

        counter = 0

        while counter < 3:
            speech_text = speech_proxy()

            if 'yes' in speech_text.outp.lower():
                sentence = nlg.dialogue['introduction'][4]
                sentence += nlg.dialogue['introduction'][5]
                sentence += nlg.dialogue['introduction'][6]
                sentence += nlg.dialogue['introduction'][7]
                sentence += nlg.dialogue['introduction'][8]
                tts_proxy(sentence)

                counter = 0

                while counter < 3:
                    speech_text = speech_proxy()

                    if 'no' in speech_text.outp.lower():
                        tts_proxy(nlg.dialogue['introduction'][9])

                        return 'predprocess'

                    elif 'yes' in speech_text.outp.lower():
                        tts_proxy(nlg.dialogue['introduction'][10])
                        raw_input('Press enter to continue...')
                        lm.write('[Pressed enter to continue]')

                        return 'predprocess'

                    else:
                        tts_proxy(nlg.dialogue['general'][0])
                        counter += 1

            elif 'no' in speech_text.outp.lower():
                tts_proxy(nlg.dialogue['introduction'][10])
                raw_input('Press enter to continue...')
                lm.write('[Pressed enter to continue]')

                return 'introduction' #USE CONTAINER STATES IN CASE OF ERRORS

            else:
                counter += 1
                if counter < 3:
                    tts_proxy(nlg.dialogue['general'][0])

        userdata.status = 'not_understood'
        return 'goodbye'


class PastImpactful(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect', 'goodbye'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state PAST_IMPACTFUL')
        lm_flow.setTableState('PAST_IMPACTFUL')

        userdata.interaction = 'grateful'

        tts_proxy(nlg.dialogue['past']['impactful'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['past']['impactful'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['impactful'][2])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['impactful'][3])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['impactful'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            return 'e_detect'

        return 'predprocess'


class PastGrateful(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect', 'goodbye'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state PAST_GRATEFUL')
        lm_flow.setTableState('PAST_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['past']['grateful'][0]
        sentence += nlg.dialogue['past']['grateful'][1]
        sentence += nlg.dialogue['past']['grateful'][2]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['past']['grateful'][3])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['grateful'][4]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['grateful'][5]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['grateful'][6])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][2])
            sentence += random.choice(nlg.dialogue['phrases'][3])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class PastAccomplishments(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state PAST_ACCOMPLISHMENTS')
        lm_flow.setTableState('PAST_ACCOMPLISHMENTS')

        userdata.task = 'present'
        userdata.interaction = 'feedback'

        tts_proxy(nlg.dialogue['past']['accomplishments'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['past']['accomplishments'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['accomplishments'][2]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['accomplishments'][3]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['accomplishments'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][0]).format(userdata.kb_args['name'])
            sentence += random.choice(nlg.dialogue['phareses'][1])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class PresentImpactful(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state PRESENT_IMPACTFUL')
        lm_flow.setTableState('PRESENT_IMPACTFUL')

        userdata.interaction = 'grateful'

        tts_proxy(nlg.dialogue['present']['impactful'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['present']['impactful'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['impactful'][2]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['impactful'][3])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['impactful'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            return 'e_detect'

        return 'predprocess'


class PresentGrateful(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state PRESENT_GRATEFUL')
        lm_flow.setTableState('PRESENT_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['present']['grateful'][0]
        sentence += nlg.dialogue['present']['grateful'][1]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['present']['grateful'][2])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['grateful'][3]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['grateful'][4]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['grateful'][5])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][2])
            sentence += random.choice(nlg.dialogue['phrases'][3])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class PresentAccomplishments(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state PRESENT_ACCOMPLISHMENTS')
        lm_flow.setTableState('PRESENT_ACCOMPLISHMENTS')

        userdata.task = 'future'
        userdata.interaction = 'feedback'

        tts_proxy(nlg.dialogue['present']['accomplishments'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['present']['accomplishments'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['accomplishments'][2]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['accomplishments'][3]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['accomplishments'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][0]).format(userdata.kb_args['name'])
            sentence += random.choice(nlg.dialogue['phrases'][1])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class FutureImpactful(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state FUTURE_IMPACTFUL')
        lm_flow.setTableState('FUTURE_IMPACTFUL')

        userdata.interaction = 'grateful'

        sentence = nlg.dialogue['future']['impactful'][0]
        sentence += nlg.dialogue['future']['impactful'][1]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['future']['impactful'][2])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['impactful'][3])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['impactful'][4])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['impactful'][5])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            return 'e_detect'

        return 'predprocess'


class FutureGrateful(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state FUTURE_GRATEFUL')
        lm_flow.setTableState('FUTURE_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['future']['grateful'][0]
        sentence += nlg.dialogue['future']['grateful'][1]
        sentence += nlg.dialogue['future']['grateful'][2]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['future']['grateful'][3])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['grateful'][4]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['grateful'][5]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['grateful'][6])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][2])
            sentence += random.choice(nlg.dialogue['phrases'][3])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class FutureAccomplishments(smach.State): #add last sentence and goodbye
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye', 'e_detect'],
                             input_keys=['kb_args', 'condition'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state FUTURE_ACCOMPLISHMENTS')
        lm_flow.setTableState('FUTURE_ACCOMPLISHMENTS')

        userdata.task = 'goodbye'
        userdata.interaction = 'feedback'

        sentence = nlg.dialogue['future']['accomplishments'][0]
        sentence += nlg.dialogue['future']['accomplishments'][1]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text.outp == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['future']['accomplishments'][2])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['accomplishments'][3]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['accomplishments'][4]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['accomplishments'][5])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = nlg.dialogue['phrases'][0][3].format(userdata.kb_args['name'])
            sentence += nlg.dialogue['phrases'][4]
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class Feedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goodbye', 'survey'],
                             input_keys=['kb_args', 'condition', 'task'],
                             output_keys=['status', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state FEEDBACK')
        lm_flow.setTableState('FEEDBACK')

        userdata.interaction = 'impactful'

        if userdata.task == 'present':
            sentence = nlg.dialogue['feedback'][0].format('past')
        elif userdata.task == 'future':
            sentence = nlg.dialogue['feedback'][0].format('present')
        if userdata.task == 'goodbye':
            sentence = nlg.dialogue['feedback'][0].format('future')

        sentence += nlg.dialogue['feedback'][1]
        sentence += nlg.dialogue['feedback'][2]
        tts_proxy(sentence)

        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['feedback'][3])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['feedback'][4])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['feedback'][5])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['feedback'][6])

        return 'survey'

class Survey(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['kb_args'],
                             output_keys=['status', 'task', 'interaction'])

    def execute(self, userdata):
        lm.write('\nExecuting state SURVEY')

        return 'predprocess'


class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['kb_args', 'status'])

    def execute(self, userdata):
        lm.write('\nExecuting state GOODBYE')
        lm_flow.setTableState('GOODBYE')

        if userdata.status == 'not_understood':
            sentence = nlg.dialogue['goodbye'][1].format(userdata.kb_args['name'])
            tts_proxy(sentence, 'goodbye')

        else:
            sentence = random.choice(nlg.dialogue['goodbye'][0]).format(userdata.kb_args['name'])
            tts_proxy(sentence, 'goodbye')

        return 'terminate'


class EmotionDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['quadrant_1', 'quadrant_2', 'quadrant_3', 'quadrant_4', 'predprocess'],
                             input_keys=['condition'],
                             output_keys=['dimension_mean'])

    def execute(self, userdata):
        lm.write('\nExecuting state E_DETECTION')
        lm_flow.setTableState('E_DETECTION')

        try:
            # getting dimension values from the frame log file
            dimensions = lm_frame.getLastLines(FACE_CHANNEL_SENSITIVITY, False)

            # converting dimension values to python lists
            dimensions = list(map(lambda x: x.replace('] [', '],['), dimensions))
            dimensions = list(map(lambda x: x.replace(' ', ''), dimensions))
            dimensions = list(map(lambda x: json.loads(x), dimensions))

            # getting the mean valued list of the dimensions
            dimensionsMean = np.mean(dimensions, axis=0)
            
            if dimensionsMean[0] > 0 and dimensionsMean[1] > 0:
                return 'quadrant_1' 

            elif dimensionsMean[0] < 0 and dimensionsMean[1] > 0:
                return 'quadrant_2'

            elif dimensionsMean[0] < 0 and dimensionsMean[1] < 0:
                return 'quadrant_3'

            elif dimensionsMean[0] > 0 and dimensionsMean[1] < 0:
                return 'quadrant_4'

            else:
                lm.write('\nWARNING: One or more dimension is equal to 0')
                lm.write('Returning to the PREDPROCESS')
                
                return 'predprocess'

        except:
            lm.write('\nWARNING: Emotion couldn\'t detected!')
            lm.write('Returning to the PREDPROCESS')

            return 'predprocess'


class Quadrant_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['kb_args', 'dimension_mean'])

    def execute(self, userdata):
        lm.write('\nExecuting state QUADRANT_1')
        lm_flow.setTableState('QUADRANT_1')

        ### THRESHOLD DIALOGUES

        return 'predprocess'


class Quadrant_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['kb_args', 'dimension_mean'])

    def execute(self, userdata):
        lm.write('\nExecuting state QUADRANT_2')
        lm_flow.setTableState('QUADRANT_2')

        ### THRESHOLD DIALOGUES

        return 'predprocess'


class Quadrant_3(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['kb_args', 'dimension_mean'])

    def execute(self, userdata):
        lm.write('\nExecuting state QUADRANT_3')
        lm_flow.setTableState('QUADRANT_3')

        ### THRESHOLD DIALOGUES

        return 'predprocess'


class Quadrant_4(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['kb_args', 'dimension_mean'])

    def execute(self, userdata):
        lm.write('\nExecuting state QUADRANT_4')
        lm_flow.setTableState('QUADRANT_4')

        ### THRESHOLD DIALOGUES

        return 'predprocess'


def main():

    try:
        lm.separate()
        lm.write('ROS STATUS', False)
        lm.separate(1)
        lm.write('ROS Master is running: ' + str(rosgraph.is_master_online()) + '\n')
        
        if rosgraph.is_master_online():
            lm.write('Running nodes:', False)
            
            for i in rosnode.get_node_names():
                lm.write(i)

        lm.separate()

        sm = smach.StateMachine(outcomes=['outcome_termination'])
        lm.write('State machine is created.')

        # use it only for using smach_viewer to view the state machine graph
        #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        #sis.start()

        # to inform states about the status ('not_understood', 'understood')
        sm.userdata.sm_status = 'understood'

        # arguments for the knowledge base
        sm.userdata.sm_kb_args = {'name': 'User'}

        # dialogue tense
        sm.userdata.sm_task = 'past'

        # dialogue interaction mode
        sm.userdata.sm_interaction = 'impactful'

        # dialogue condition (c1: only script, c2: script with FaceChannel, c3: script with Continual Learning model)
        sm.userdata.sm_condition = ''

        # mean value of the dimensions of the frames for arousal valence calculation
        sm.userdata.sm_dimension_mean = []

        # defining states for smach
        with sm:
            smach.StateMachine.add('PREDPROCESS', PredProcess(),
                                   transitions={'introduction':'INTRODUCTION',
                                                'goodbye':'GOODBYE',
                                                'past_impactful':'PAST_IMPACTFUL',
                                                'past_grateful':'PAST_GRATEFUL',
                                                'past_accomplishments':'PAST_ACCOMPLISHMENTS',
                                                'present_impactful':'PRESENT_IMPACTFUL',
                                                'present_grateful':'PRESENT_GRATEFUL',
                                                'present_accomplishments':'PRESENT_ACCOMPLISHMENTS',
                                                'future_impactful':'FUTURE_IMPACTFUL',
                                                'future_grateful':'FUTURE_GRATEFUL',
                                                'future_accomplishments':'FUTURE_ACCOMPLISHMENTS',
                                                'feedback':'FEEDBACK'},
                                   remapping={'status':'sm_status',
                                              'kb_args':'sm_kb_args',
                                              'task':'sm_task',
                                              'condition':'sm_condition',
                                              'interaction':'sm_interaction'})
            
            smach.StateMachine.add('INTRODUCTION', Introduction(),
                                   transitions={'predprocess':'PREDPROCESS',
                                                'goodbye':'GOODBYE',
                                                'introduction':'INTRODUCTION'},
                                   remapping={'status':'sm_status',
                                              'kb_args':'sm_kb_args'})

            states =    {'PAST_IMPACTFUL':PastImpactful(),
                        'PAST_GRATEFUL':PastGrateful(),
                        'PAST_ACCOMPLISHMENTS':PastAccomplishments(),
                        'PRESENT_IMPACTFUL':PresentImpactful(),
                        'PRESENT_GRATEFUL':PresentGrateful(),
                        'PRESENT_ACCOMPLISHMENTS':PresentAccomplishments(),
                        'FUTURE_IMPACTFUL':FutureImpactful(),
                        'FUTURE_GRATEFUL':FutureGrateful(),
                        'FUTURE_ACCOMPLISHMENTS':FutureAccomplishments()}

            for i in states :
                smach.StateMachine.add(i, states[i],
                                    transitions={'predprocess':'PREDPROCESS',
                                                 'goodbye':'GOODBYE',
                                                 'e_detect':'E_DETECT'},
                                    remapping={'status':'sm_status',
                                                'kb_args':'sm_kb_args',
                                                'condition':'sm_condition',
                                                'task':'sm_task',
                                                'interaction':'sm_interaction'})

            smach.StateMachine.add('FEEDBACK', Feedback(),
                                   transitions={'goodbye':'GOODBYE',
                                                'survey':'SURVEY'},
                                   remapping={'status':'sm_status',
                                              'kb_args':'sm_kb_args',
                                              'task':'sm_task',
                                              'interaction':'sm_interaction'})

            smach.StateMachine.add('SURVEY', Survey(),
                                   transitions={'predprocess':'PREDPROCESS'},
                                   remapping={'status':'sm_status',
                                              'kb_args':'sm_kb_args',
                                              'task':'sm_task',
                                              'interaction':'sm_interaction'})

            smach.StateMachine.add('GOODBYE', Goodbye(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'kb_args':'sm_kb_args',
                                              'status':'sm_status'})

            smach.StateMachine.add('E_DETECT', EmotionDetection(),
                                   transitions={'quadrant_1':'QUADRANT_1',
                                                'quadrant_2':'QUADRANT_2',
                                                'quadrant_3':'QUADRANT_3',
                                                'quadrant_4':'QUADRANT_4',
                                                'predprocess':'PREDPROCESS'},
                                   remapping={'condition':'sm_condition'})

            states =    {'QUADRANT_1': Quadrant_1(),
                         'QUADRANT_2': Quadrant_2(),
                         'QUADRANT_3': Quadrant_3(),
                         'QUADRANT_4': Quadrant_4()}

            for i in states :
                smach.StateMachine.add(i, states[i],
                                    transitions={'predprocess':'PREDPROCESS'},
                                    remapping={'kb_args':'sm_kb_args',
                                               'dimension_mean':'sm_dimension_mean'})

        lm.write('State machine is executed')
        outcome = sm.execute()

        rospy.spin()

    except Exception as e:
        lm.write(e.message)

    finally:
        #sis.stop()
        
        lm_flow.printTable()
        lm.close()
        lm_flow.close()
        

if __name__ == '__main__':
    main()