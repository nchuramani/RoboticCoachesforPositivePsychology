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
lm_arousal_valence_CL = LogManager('arousal_valence_CL')

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


def tts_proxy(inp_text, behavior='normal_talk'):
    tts_text = tts_ros_proxy(inp_text, behavior)
    lm_flow.tableAddRow(['PEPPER', inp_text])

def speech_proxy(timeout='', dimensions=False, experimenterAnswer=False, condition='c2'):
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
    lm_flow.tableAddRow([name, answer])

    # if the arousal-valence values of the speech requested
    if dimensions:
        lm.write('\nArousal Valence frames start time:' + str(start))
        lm.write('Arousal Valence frames stop time:' + str(stop) + '\n')
        if condition == 'c2':
            return speech_text, lm_arousal_valence.getTimeIntervalLines(start, stop, False, lines=rospy.get_param('arousal_valence'))
        elif condition == 'c3':
            return speech_text, lm_arousal_valence_CL.getTimeIntervalLines(start, stop, False, lines=rospy.get_param('arousal_valence_cl'))


    return answer


#initiazes states
def setStateInfo(stateName):
    lm.write('\nExecuting state ' + stateName)
    lm_flow.setTableState(stateName)

    lm_arousal_valence.separate(1)
    lm_arousal_valence.write(stateName, False, False)
    lm_arousal_valence.separate(1)
    lm_arousal_valence_CL.separate(1)
    lm_arousal_valence_CL.write(stateName, False, False)
    lm_arousal_valence_CL.separate(1)
    rospy.set_param('current_state', stateName)

def emotionDetection(dimensions, state=""):
    lm.write('Emotion detection function started')

    try:
        # converting dimension values to python lists
        dimensions = list(map(lambda x: x.replace('] [', '],['), dimensions))
        dimensions = list(map(lambda x: x.replace(' ', ''), dimensions))
        dimensions = list(map(lambda x: json.loads(x), dimensions))

        # getting the mean valued list of the dimensions
        dimensionsMean = np.mean(dimensions, axis=0)
        lm.write('\nArousal-Valence average values: ' + str(dimensionsMean) + '\n')

        arousal, valence = dimensionsMean[0][0][0], dimensionsMean[1][0][0]
        if (arousal > -0.1 and arousal < 0.1) and (valence > -0.1 and valence < 0.1):
            lm.write('Neutral is running...')
            lm.write('\nEmotion: <NEUTRAL>')
            tts_proxy(nlg.dialogue['emotion'][8], 'neutral')

            return
        if arousal > 0 and valence > 0:
            lm.write('QUADRANT 1 is running...')

            # sentences = nlg.dialogue['emotion'][1] #happy
            # sentences.extend(nlg.dialogue['emotion'][1]) #surprise

            # if arousal > 0.5:
            #     lm.write('\nEmotion: <SURPRISE>')
            #
            #     tts_proxy(random.choice(nlg.dialogue['emotion'][5]), 'positive')
            #
            # else:
            lm.write('\nEmotion: <POSITIVE(HAPPY)>')
            if state == 'past' or state == 'present':
                tts_proxy(random.choice(nlg.dialogue['emotion'][2]), 'positive')
            else:
                tts_proxy(random.choice(nlg.dialogue['emotion'][3]), 'positive')

            return

        elif arousal > 0 and valence < 0:
            lm.write('QUADRANT 2 is running...')

            if state == 'past' or state == 'present':
                sentences = nlg.dialogue['emotion'][0] #sad
                sentences.extend(nlg.dialogue['emotion'][4]) #angry
                sentences.append(nlg.dialogue['emotion'][6]) #disgust
            else:
                sentences = nlg.dialogue['emotion'][1]  # sad
                sentences.extend(nlg.dialogue['emotion'][5])  # angry
                sentences.append(nlg.dialogue['emotion'][7])  # disgust1

            # if arousal > 0.5 and valence < -0.3:
            #     lm.write('\nEmotion: <FEAR>')
            #
            #     tts_proxy(random.choice(nlg.dialogue['emotion'][3]), 'negative')
            #
            # else:
            lm.write('\nEmotion: <NEGATIVE(SAD, ANGRY, DISGUST)>')

            tts_proxy(random.choice(sentences), 'negative')

            return

        elif arousal < 0 and valence < 0:
            lm.write('QUADRANT 3 is running...')

            lm.write('\nEmotion: <SAD>')
            if state == 'past' or state == 'present':
                tts_proxy(random.choice(nlg.dialogue['emotion'][0]), 'negative')
            else:
                tts_proxy(random.choice(nlg.dialogue['emotion'][1]), 'negative')


            return


        elif arousal < 0 and valence > 0:
            lm.write('QUADRANT 4 is running...')

            lm.write('\nEmotion: <HAPPY>')
            if state == 'past' or state == 'present':
                tts_proxy(random.choice(nlg.dialogue['emotion'][2]), 'positive')
            else:
                tts_proxy(random.choice(nlg.dialogue['emotion'][3]), 'positive')

            return

        else:
            lm.write('\nWARNING: One or more dimensions are equal to 0. Returning...')

            return

    except:
        lm.write('\nWARNING: Emotion couldn\'t detected! Returning...')


class PredProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                        outcomes=['introduction',
                                'past_impactful', 'past_grateful', 'past_accomplishments',
                                'present_impactful', 'present_grateful', 'present_accomplishments',
                                'future_impactful', 'future_grateful', 'future_accomplishments',
                                'feedback', 'goodbye'],
                        input_keys=['task', 'interaction', 'condition'],
                        output_keys=['condition'])

        self.first_time = True

        #choosing the condition randomly
        conditions = ['c1','c2','c3']
        random.shuffle(conditions)

        self.task_condition = {'past':conditions[0], 'present':conditions[1], 'future':conditions[2]}

        self.condition = ''

    def execute(self, userdata):
        setStateInfo('PREDPROCESS')

        if self.first_time:
            # tts_proxy(nlg.dialogue['introduction'][0])
            tts_proxy(nlg.dialogue['introduction'][0], 'hello')
            tts_proxy(nlg.dialogue['introduction'][1])

            lm.write('User name is initialized as [' + name + ']')

            self.first_time = False

            return 'introduction'

        else:
            if userdata.task in self.task_condition.keys():
                self.condition = self.task_condition[userdata.task]

            # if userdata.condition == 'c2' and self.condition == 'c3':
            #     rospy.set_param('arousal_valence', [])

            userdata.condition = self.condition
            lm_flow.setTableCondition(self.condition.upper())

            if userdata.interaction == 'impactful' and userdata.task != 'goodbye':
                lm.write('Dialogue condition is randomly selected: [' + self.condition.upper() + ']')

            if userdata.interaction == 'feedback':
                return 'feedback'

            if userdata.task == 'goodbye':
                return 'goodbye'

            return userdata.task + '_' + userdata.interaction


class Introduction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess', 'introduction'])

    def execute(self, userdata):
        setStateInfo('INTRODUCTION')

        sentence = nlg.dialogue['introduction'][2]
        sentence += nlg.dialogue['introduction'][3]
        tts_proxy(sentence, 'questioning')

        while True:

            speech_text = speech_proxy(experimenterAnswer=True)

            if any(i in speech_text.lower() for i in nlg.dialogue['yes/no'][0]):
                sentence = nlg.dialogue['introduction'][4]
                sentence += nlg.dialogue['introduction'][5]
                sentence += nlg.dialogue['introduction'][6]
                sentence += nlg.dialogue['introduction'][7]
                sentence += nlg.dialogue['introduction'][8].format(EXPERIMENTER)
                tts_proxy(sentence, 'questioning')

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
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PAST_IMPACTFUL')

        userdata.interaction = 'grateful'

        tts_proxy(nlg.dialogue['past']['impactful'][0])

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['past']['impactful'][1])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['past']['impactful'][2])
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="past")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['past']['impactful'][3])
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="past")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['past']['impactful'][4])


            count += 1

            if count == 1:
                tts_proxy(nlg.dialogue['past']['impactful'][5].format('second'))
            # elif count == 2:
            #     tts_proxy(nlg.dialogue['past']['impactful'][5].format('third'))

        return 'predprocess'


class PastGrateful(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PAST_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['past']['grateful'][0]
        sentence += nlg.dialogue['past']['grateful'][1]
        sentence += nlg.dialogue['past']['grateful'][2]
        tts_proxy(sentence)

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['past']['grateful'][3])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['past']['grateful'][4]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="past")

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['past']['grateful'][5]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                sentence = random.choice(nlg.dialogue['phrases'][2])
                tts_proxy(sentence)

                emotionDetection(userdata.dimensions, state="past")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['past']['grateful'][6])


            count += 1

            if count != 2:
                tts_proxy(nlg.dialogue['past']['grateful'][7])

        return 'predprocess'


class PastAccomplishments(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PAST_ACCOMPLISHMENTS')

        userdata.task = 'present'
        userdata.interaction = 'feedback'

        tts_proxy(nlg.dialogue['past']['accomplishments'][0])

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['past']['accomplishments'][1])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['past']['accomplishments'][2]))
            speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['past']['accomplishments'][3]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="past")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['past']['accomplishments'][4])

            if userdata.condition == 'c2' or userdata.condition == 'c3':
                sentence = random.choice(nlg.dialogue['phrases'][0])
                # sentence += random.choice(nlg.dialogue['phrases'][0])
                tts_proxy(sentence)

                emotionDetection(userdata.dimensions, state="past")

            count += 1

            if count != 2:
                tts_proxy(nlg.dialogue['past']['accomplishments'][5])

        return 'predprocess'


class PresentImpactful(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PRESENT_IMPACTFUL')

        userdata.interaction = 'grateful'

        tts_proxy(nlg.dialogue['present']['impactful'][0])

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['present']['impactful'][1])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['present']['impactful'][2])
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="present")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['present']['impactful'][3])
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="present")
            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['present']['impactful'][4])

            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="present")

            count += 1

            if count == 1:
                tts_proxy(nlg.dialogue['present']['impactful'][5].format('second'))
            # elif count == 2:
            #     tts_proxy(nlg.dialogue['present']['impactful'][5].format('third'))

        return 'predprocess'


class PresentGrateful(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PRESENT_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['present']['grateful'][0]
        sentence += nlg.dialogue['present']['grateful'][1]
        tts_proxy(sentence)

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['present']['grateful'][2])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['present']['grateful'][3]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="present")

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['present']['grateful'][4]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="present")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['present']['grateful'][5])

            if userdata.condition == 'c2' or userdata.condition == 'c3':
                sentence = random.choice(nlg.dialogue['phrases'][2])
                tts_proxy(sentence)

                emotionDetection(userdata.dimensions, state="present")

            count += 1

            if count != 2:
                tts_proxy(nlg.dialogue['present']['grateful'][6])

        return 'predprocess'


class PresentAccomplishments(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('PRESENT_ACCOMPLISHMENTS')

        userdata.task = 'future'
        userdata.interaction = 'feedback'

        tts_proxy(nlg.dialogue['present']['accomplishments'][0])

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['present']['accomplishments'][1])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['present']['accomplishments'][2]))
            speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['present']['accomplishments'][3]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="present")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['present']['accomplishments'][4])

            if userdata.condition == 'c2' or userdata.condition == 'c3':
                sentence = random.choice(nlg.dialogue['phrases'][0])
                tts_proxy(sentence)
                emotionDetection(userdata.dimensions, state="present")

            count += 1

            if count != 2:
                tts_proxy(nlg.dialogue['present']['accomplishments'][5])

        return 'predprocess'


class FutureImpactful(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('FUTURE_IMPACTFUL')

        userdata.interaction = 'grateful'

        sentence = nlg.dialogue['future']['impactful'][0]
        sentence += nlg.dialogue['future']['impactful'][1]
        tts_proxy(sentence)

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['future']['impactful'][2])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['future']['impactful'][3])
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="future")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['future']['impactful'][4])
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="future")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['future']['impactful'][5])
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="future")

            count += 1

            if count == 1:
                tts_proxy(nlg.dialogue['future']['impactful'][6].format('second'))
            # elif count == 2:
            #     tts_proxy(nlg.dialogue['future']['impactful'][6].format('third'))

        return 'predprocess'


class FutureGrateful(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('FUTURE_GRATEFUL')

        userdata.interaction = 'accomplishments'

        sentence = nlg.dialogue['future']['grateful'][0]
        sentence += nlg.dialogue['future']['grateful'][1]
        sentence += nlg.dialogue['future']['grateful'][2]
        tts_proxy(sentence)

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['future']['grateful'][3])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['future']['grateful'][4]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="future")

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['future']['grateful'][5]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="future")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['future']['grateful'][6])

            if userdata.condition == 'c2' or userdata.condition == 'c3':
                sentence = random.choice(nlg.dialogue['phrases'][3])
                tts_proxy(sentence)
                emotionDetection(userdata.dimensions, state="future")

            count += 1

            if count != 2:
                tts_proxy(nlg.dialogue['future']['grateful'][7])

        return 'predprocess'


class FutureAccomplishments(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['predprocess'],
                             input_keys=['condition', 'dimensions'],
                             output_keys=['task', 'interaction', 'dimensions'])

    def execute(self, userdata):
        setStateInfo('FUTURE_ACCOMPLISHMENTS')

        userdata.task = 'goodbye'
        userdata.interaction = 'feedback'

        sentence = nlg.dialogue['future']['accomplishments'][0]
        sentence += nlg.dialogue['future']['accomplishments'][1]
        tts_proxy(sentence)

        count = 0

        while count < 2:

            speech_text = speech_proxy(TIMEOUT)

            while speech_text == '<NOT_RESPONDED>':
                tts_proxy(nlg.dialogue['future']['accomplishments'][2])

                speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['future']['accomplishments'][3]))
            speech_text = speech_proxy()

            time.sleep(DELAY)

            tts_proxy(random.choice(nlg.dialogue['future']['accomplishments'][4]))
            speech_text = speech_proxy(dimensions=True, condition=userdata.condition)
            userdata.dimensions = speech_text[1]
            if userdata.condition == 'c2' or userdata.condition == 'c3':
                emotionDetection(userdata.dimensions, state="future")

            time.sleep(DELAY)

            tts_proxy(nlg.dialogue['future']['accomplishments'][5])

            if userdata.condition == 'c2' or userdata.condition == 'c3':
                sentence = random.choice(nlg.dialogue['phrases'][1])
                # sentence += nlg.dialogue['phrases'][1]
                tts_proxy(sentence)

                emotionDetection(userdata.dimensions, state="future")

            count += 1

            if count != 2:
                tts_proxy(nlg.dialogue['future']['accomplishments'][6])

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
        tts_proxy(sentence, 'questioning')

        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['feedback'][3])
        speech_text = speech_proxy()

        time.sleep(DELAY)

        tts_proxy(nlg.dialogue['feedback'][4], 'questioning')
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

        tts_proxy(nlg.dialogue['goodbye'].format(EXPERIMENTER), 'goodbye')

        return 'terminate'


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
                                                'feedback':'FEEDBACK',
                                                'goodbye':'GOODBYE'},
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
                                    transitions={'predprocess':'PREDPROCESS'},
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