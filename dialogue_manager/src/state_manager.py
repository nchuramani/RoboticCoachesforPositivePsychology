#!/usr/bin/env python

import nlg

import time
import os
import random
import sys

from datetime import datetime

import json
import numpy as np

import smach
import smach_ros

from log_manager import LogManager

import rospy
from dialogue_manager.srv import *

import rosnode
import rosgraph

# changing directory to the file's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# getting the name of the user
name = str(rospy.get_param('name'))

# getting the values from the config file. they can be modified.
with open('./config.txt', 'r') as f:
    fLines = f.readlines()

    TIMEOUT = fLines[2].split('=')[1].strip()
    DELAY = float(fLines[3].split('=')[1].strip())
    EXPERIMENTER = fLines[11].split('=')[1].strip()

# creating the main log file
lm = LogManager('main')

# creating log file for the state transitions and dialogue table
lm_flow = LogManager('flow')
lm_flow.createTable(['State', 'Condition', 'Speaker', 'Dialogue'])

# creating/connecting log file to get the frames' dimensional outputs
lm_arousal_valence = LogManager('arousal_valence')

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

def speech_proxy(timeout='', dimensions=False, experimenterAnswer=False):
    counter = 0

    # asks for answer if not understands until 3 times.
    while counter < 3:
        start = datetime.now()
        speech_text = speech_ros_proxy(timeout, '')
        stop = datetime.now()

        if speech_text.outp == '<NOT_UNDERSTOOD>':
            counter += 1
            lm.write('Speech could not be understood ' + str(counter) + ' times.')
            if counter < 3:
                tts_proxy(nlg.dialogue['general'][0])

        else:
            break

    answer = speech_text.outp

    # if not understood 3 times.
    if counter == 3:
        # asks the experimenter to type the answer of the user instead of asking again
        if experimenterAnswer:
            lm.write('Not understood too many times. Waiting for the experimenter to type the answer...')
            answer = raw_input('{}, please type the user\'s answer: '.format(EXPERIMENTER))
            lm.write('[Typed answer]')

        else:
            lm.write('Not understood too many times. Waiting for the experimenter to press enter to continue...')
            raw_input('Press enter to continue...')
            lm.write('[Pressed enter to continue]')

    # adds dialogue to the table of flow log file
    lm_flow.tableAddRow([name.upper(), answer])

    # if the arousal-valence values of the speech requested
    if dimensions:
        lm.write('\nArousal Valence frames start time:' + str(start))
        lm.write('Arousal Valence frames stop time:' + str(stop) + '\n')
        return speech_text, lm_arousal_valence.getTimeIntervalLines(start, stop, False, lines=rospy.get_param('arousal_valence'))

    return answer


#initiazes states
def setStateInfo(stateName):
    lm.write('\nExecuting state ' + stateName)
    lm_flow.setTableState(stateName)

    lm_arousal_valence.separate(1)
    lm_arousal_valence.write(stateName, False, False)
    lm_arousal_valence.separate(1)

    rospy.set_param('current_state', stateName)

class PredProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                        outcomes=['introduction',
                                'past_impactful', 'past_grateful', 'past_accomplishments',
                                'present_impactful', 'present_grateful', 'present_accomplishments',
                                'future_impactful', 'future_grateful', 'future_accomplishments',
                                'feedback'],
                        input_keys=['task', 'interaction'],
                        output_keys=['condition'])

        self.first_time = True

        #choosing the condition randomly
        conditions = ['c1','c2','c3']
        random.shuffle(conditions)

        self.task_condition = {'past':conditions[0], 'present':conditions[1], 'future': conditions[2]}

    def execute(self, userdata):
        setStateInfo('PREDPROCESS')

        if self.first_time:
            tts_proxy(nlg.dialogue['introduction'][0])
            tts_proxy(nlg.dialogue['introduction'][1])

            lm.write('User name is initialized as [' + name.upper() + ']')

            self.first_time = False

            return 'introduction'

        else:
            try:
                condition = self.task_condition[userdata.task]
                userdata.condition = condition
                lm_flow.setTableCondition(condition.upper())

            except:
                pass

            if userdata.interaction == 'impactful':
                lm.write('Dialogue condition is randomly selected: [' + condition.upper() + ']')

            if userdata.interaction == 'feedback':
                return 'feedback'

            return userdata.task + '_' + userdata.interaction


class Introduction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'introduction'])
                             
    def execute(self, userdata):
        setStateInfo('INTRODUCTION')
        
        sentence = nlg.dialogue['introduction'][2]
        sentence += nlg.dialogue['introduction'][3]
        tts_proxy(sentence)

        while True:

            speech_text = speech_proxy(experimenterAnswer=True)

            if any(i in speech_text.lower() for i in nlg.dialogue['yes/no'][0]):
                sentence = nlg.dialogue['introduction'][4]
                sentence += nlg.dialogue['introduction'][5]
                sentence += nlg.dialogue['introduction'][6]
                sentence += nlg.dialogue['introduction'][7]
                sentence += nlg.dialogue['introduction'][8].format(EXPERIMENTER)
                tts_proxy(sentence)

                while True:

                    speech_text = speech_proxy(experimenterAnswer=True)

                    if any(i in speech_text.lower() for i in nlg.dialogue['yes/no'][1]):
                        tts_proxy(nlg.dialogue['introduction'][9])

                        return 'predprocess'

                    elif any(i in speech_text.lower() for i in nlg.dialogue['yes/no'][0]):
                        tts_proxy(nlg.dialogue['introduction'][10].format(EXPERIMENTER))
                        raw_input('Press enter to continue...')
                        lm.write('[Pressed enter to continue]')

                        return 'predprocess'

                    else:
                        tts_proxy(nlg.dialogue['general'][1])

            elif any(i in speech_text.lower() for i in nlg.dialogue['yes/no'][1]):
                tts_proxy(nlg.dialogue['introduction'][10].format(EXPERIMENTER))
                raw_input('Press enter to continue...')
                lm.write('[Pressed enter to continue]')

                return 'introduction'

            else:
                tts_proxy(nlg.dialogue['general'][1])


class PastImpactful(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PAST_IMPACTFUL')

        userdata.interaction = 'grateful'

        tts_proxy(nlg.dialogue['past']['impactful'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['past']['impactful'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['impactful'][2])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['impactful'][3])
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['impactful'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            return 'e_detect'

        return 'predprocess'


class PastGrateful(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PAST_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['past']['grateful'][0]
        sentence += nlg.dialogue['past']['grateful'][1]
        sentence += nlg.dialogue['past']['grateful'][2]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['past']['grateful'][3])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['grateful'][4]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['grateful'][5]))
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['grateful'][6])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][2])
            sentence += random.choice(nlg.dialogue['phrases'][3])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class PastAccomplishments(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PAST_ACCOMPLISHMENTS')

        userdata.task = 'present'
        userdata.interaction = 'feedback'

        tts_proxy(nlg.dialogue['past']['accomplishments'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['past']['accomplishments'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['accomplishments'][2]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['past']['accomplishments'][3]))
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['past']['accomplishments'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][0])
            sentence += random.choice(nlg.dialogue['phareses'][1])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class PresentImpactful(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PRESENT_IMPACTFUL')

        userdata.interaction = 'grateful'

        tts_proxy(nlg.dialogue['present']['impactful'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['present']['impactful'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['impactful'][2]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['impactful'][3])
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['impactful'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            return 'e_detect'

        return 'predprocess'


class PresentGrateful(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PRESENT_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['present']['grateful'][0]
        sentence += nlg.dialogue['present']['grateful'][1]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['present']['grateful'][2])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['grateful'][3]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['grateful'][4]))
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['grateful'][5])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][2])
            sentence += random.choice(nlg.dialogue['phrases'][3])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class PresentAccomplishments(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PRESENT_ACCOMPLISHMENTS')

        userdata.task = 'future'
        userdata.interaction = 'feedback'

        tts_proxy(nlg.dialogue['present']['accomplishments'][0])

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['present']['accomplishments'][1])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['accomplishments'][2]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['present']['accomplishments'][3]))
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['present']['accomplishments'][4])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][0])
            sentence += random.choice(nlg.dialogue['phrases'][1])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class FutureImpactful(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('FUTURE_IMPACTFUL')

        userdata.interaction = 'grateful'

        sentence = nlg.dialogue['future']['impactful'][0]
        sentence += nlg.dialogue['future']['impactful'][1]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['future']['impactful'][2])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['impactful'][3])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['impactful'][4])
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['impactful'][5])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            return 'e_detect'

        return 'predprocess'


class FutureGrateful(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('FUTURE_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['future']['grateful'][0]
        sentence += nlg.dialogue['future']['grateful'][1]
        sentence += nlg.dialogue['future']['grateful'][2]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['future']['grateful'][3])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['grateful'][4]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['grateful'][5]))
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['grateful'][6])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = random.choice(nlg.dialogue['phrases'][2])
            sentence += random.choice(nlg.dialogue['phrases'][3])
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class FutureAccomplishments(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'e_detect'],
                             input_keys=['condition'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('FUTURE_ACCOMPLISHMENTS')

        userdata.task = 'goodbye'
        userdata.interaction = 'feedback'

        sentence = nlg.dialogue['future']['accomplishments'][0]
        sentence += nlg.dialogue['future']['accomplishments'][1]
        tts_proxy(sentence)

        speech_text = speech_proxy(TIMEOUT)

        while speech_text == '<NOT_RESPONDED>':
            tts_proxy(nlg.dialogue['future']['accomplishments'][2])

            speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['accomplishments'][3]))
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(random.choice(nlg.dialogue['future']['accomplishments'][4]))
        speech_text = speech_proxy(dimensions=True)
        userdata.dimensions = speech_text[1]

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['future']['accomplishments'][5])

        if userdata.condition == 'c2' or userdata.condition == 'c3':
            sentence = nlg.dialogue['phrases'][0][3]
            sentence += nlg.dialogue['phrases'][4]
            tts_proxy(sentence)

            return 'e_detect'

        return 'predprocess'


class Feedback(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['survey'],
                             input_keys=['condition', 'task'],
                             output_keys=['interaction'])

    def execute(self, userdata):
        setStateInfo('FEEDBACK')

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
                             output_keys=['task', 'interaction'])

    def execute(self, userdata):
        setStateInfo('SURVEY')

        raw_input('Press enter when the survey is finished...')
        lm.write('[Survey is finished and pressed enter]')

        return 'predprocess'


class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'])

    def execute(self, userdata):
        setStateInfo('GOODBYE')

        tts_proxy(nlg.dialogue['goodbye'][0])

        return 'terminate'


class EmotionDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['quadrant_1', 'quadrant_2', 'quadrant_3', 'quadrant_4', 'predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['dimension_mean'])

    def execute(self, userdata):
        setStateInfo('E_DETECTION')

        try:
            # getting dimension values
            dimensions = userdata.dimensions

            # converting dimension values to python lists
            dimensions = list(map(lambda x: x.replace('] [', '],['), dimensions))
            dimensions = list(map(lambda x: x.replace(' ', ''), dimensions))
            dimensions = list(map(lambda x: json.loads(x), dimensions))

            # getting the mean valued list of the dimensions
            dimensionsMean = np.mean(dimensions, axis=0)
            userdata.dimension_mean = dimensionsMean
            lm.write('\nArousal-Valence average values: ' + str(dimensionsMean) + '\n')
            
            if dimensionsMean[0] > 0 and dimensionsMean[1] > 0:
                return 'quadrant_1' 

            elif dimensionsMean[0] > 0 and dimensionsMean[1] < 0:
                return 'quadrant_2'

            elif dimensionsMean[0] < 0 and dimensionsMean[1] < 0:
                return 'quadrant_3'

            elif dimensionsMean[0] < 0 and dimensionsMean[1] > 0:
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
                             input_keys=['dimension_mean'])

    def execute(self, userdata):
        setStateInfo('QUADRANT_1')

        dimensionsMean = userdata.dimension_mean
        arousal, valence = dimensionsMean[0][0][0], dimensionsMean[1][0][0]

        sentences = nlg.dialogue['emotion'][1] #happy
        sentences.extend(nlg.dialogue['emotion'][5]) #surprise

        if arousal > 0.5:
            lm.write('\nEmotion: <SURPRISE>')

            tts_proxy(random.choice(nlg.dialogue['emotion'][5]))

        else:
            lm.write('\nEmotion: <POSITIVE(HAPPY, SURPRISE)>')

            tts_proxy(random.choice(sentences))

        return 'predprocess'


class Quadrant_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['dimension_mean'])

    def execute(self, userdata):
        setStateInfo('QUADRANT_2')

        dimensionsMean = userdata.dimension_mean
        arousal, valence = dimensionsMean[0][0][0], dimensionsMean[1][0][0]

        sentences = nlg.dialogue['emotion'][0] #sad
        sentences.extend(nlg.dialogue['emotion'][2]) #angry
        sentences.append(nlg.dialogue['emotion'][4]) #disgust

        if arousal > 0.5 and valence < -0.3:
            lm.write('\nEmotion: <FEAR>')

            tts_proxy(random.choice(nlg.dialogue['emotion'][3]))

        else:
            lm.write('\nEmotion: <NEGATIVE(SAD, ANGRY, DISGUST)>')

            tts_proxy(random.choice(sentences))

        return 'predprocess'


class Quadrant_3(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['dimension_mean'])

    def execute(self, userdata):
        setStateInfo('QUADRANT_3')
        
        lm.write('\nEmotion: <SAD>')

        tts_proxy(random.choice(nlg.dialogue['emotion'][0]))
            
        return 'predprocess'


class Quadrant_4(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess'],
                             input_keys=['dimension_mean'])

    def execute(self, userdata):
        setStateInfo('QUADRANT_4')

        lm.write('\nEmotion: <HAPPY>')

        tts_proxy(random.choice(nlg.dialogue['emotion'][1]))

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
        # requires smach_viewer. see: http://wiki.ros.org/smach_viewer
        #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        #sis.start()

        # dialogue tense
        sm.userdata.sm_task = 'past'

        # dialogue interaction mode
        sm.userdata.sm_interaction = 'impactful'

        # dialogue condition (c1: only script, c2: script with FaceChannel, c3: script with Continual Learning model)
        sm.userdata.sm_condition = ''

        # the angle of the mean value of the dimensions of the frames for arousal valence calculation
        sm.userdata.sm_dimension_mean = []

        # holds the user's speech frames' dimensional values
        sm.userdata.sm_dimensions = []

        # defining states for smach
        with sm:
            smach.StateMachine.add('PREDPROCESS', PredProcess(),
                                   transitions={'introduction':'INTRODUCTION',
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
                                   remapping={'task':'sm_task',
                                              'condition':'sm_condition',
                                              'interaction':'sm_interaction'})
            
            smach.StateMachine.add('INTRODUCTION', Introduction(),
                                   transitions={'predprocess':'PREDPROCESS',
                                                'introduction':'INTRODUCTION'})

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
                                                 'e_detect':'E_DETECT'},
                                    remapping={'condition':'sm_condition',
                                                'task':'sm_task',
                                                'interaction':'sm_interaction',
                                                'dimensions':'sm_dimensions'})

            smach.StateMachine.add('FEEDBACK', Feedback(),
                                   transitions={'survey':'SURVEY'},
                                   remapping={'task':'sm_task',
                                              'interaction':'sm_interaction'})

            smach.StateMachine.add('SURVEY', Survey(),
                                   transitions={'predprocess':'PREDPROCESS'},
                                   remapping={'task':'sm_task',
                                              'interaction':'sm_interaction'})

            smach.StateMachine.add('GOODBYE', Goodbye(),
                                   transitions={'terminate':'outcome_termination'})

            smach.StateMachine.add('E_DETECT', EmotionDetection(),
                                   transitions={'quadrant_1':'QUADRANT_1',
                                                'quadrant_2':'QUADRANT_2',
                                                'quadrant_3':'QUADRANT_3',
                                                'quadrant_4':'QUADRANT_4',
                                                'predprocess':'PREDPROCESS'},
                                   remapping={'condition':'sm_condition',
                                              'dimensions':'sm_dimensions',
                                              'dimension_mean':'sm_dimension_mean'})

            states =    {'QUADRANT_1': Quadrant_1(),
                         'QUADRANT_2': Quadrant_2(),
                         'QUADRANT_3': Quadrant_3(),
                         'QUADRANT_4': Quadrant_4()}

            for i in states :
                smach.StateMachine.add(i, states[i],
                                    transitions={'predprocess':'PREDPROCESS'},
                                    remapping={'dimension_mean':'sm_dimension_mean'})

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