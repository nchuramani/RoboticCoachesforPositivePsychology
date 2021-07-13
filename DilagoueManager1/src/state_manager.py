#!/usr/bin/env python

import random
import sys
import nlg

import sqlite3
import os

import smach
import smach_ros

import rospy
from dialogue_manager.srv import *

rospy.init_node('state_manager', anonymous=True)
# arguments for the knowledge base
rospy.wait_for_service('speech_recognizer_srv')
rospy.wait_for_service('text_to_speech_srv')

try:
    tts_proxy = rospy.ServiceProxy('text_to_speech_srv', tts_srv)
    speech_proxy = rospy.ServiceProxy('speech_recognizer_srv', tts_srv)

except rospy.ServiceException as e:
    print('service call failed {}'.format(e))

class PredProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                        outcomes=['hello', 'day', 'goodbye'],
                        input_keys=['kb_args', 'db_con', 'db_cursor'],
                        output_keys=['kb_args', 'status'])

        self.first_time = True

    def execute(self, userdata):
        rospy.loginfo('Executing state PREDPROCESS')

        if self.first_time:
            sentence = random.choice(nlg.dialogue['learn_name'][:3])
            tts_proxy(sentence)
            print 'Pepper:', sentence

            name = raw_input('Enter your first name: ')

            userdata.kb_args['name'] = name
            userdata.db_cursor.execute('INSERT INTO userdata VALUES(?,?,?)', (name, '', 0))
            userdata.db_con.commit()

            self.first_time = False

            return 'hello'

        else:
            sentence = random.choice(nlg.dialogue['get_command']).format(userdata.kb_args['name'])
            tts_proxy(sentence)
            print 'Pepper:', sentence

            counter = 0

            while counter < 3:
                speech_text = speech_proxy('')
                print userdata.kb_args['name'] + ":", speech_text.outp

                if 'day' in speech_text.outp.lower():
                    return 'day'

                elif 'goodbye' in speech_text.outp.lower():
                    return 'goodbye'

                else:
                    sentence = random.choice(nlg.dialogue['not_understood']).format(userdata.kb_args['name'])
                    tts_proxy(sentence)
                    print 'Pepper:', sentence
                    counter += 1

            userdata.status = 'not_understood'
            return 'goodbye'

class Hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye'],
                             input_keys=['kb_args', 'db_con', 'db_cursor'],
                             output_keys=['status', 'kb_args'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HELLO')

        sentence = random.choice(nlg.dialogue['after_meeting']).format(userdata.kb_args['name']) + ' ' + random.choice(nlg.dialogue['ask_id']).format(userdata.kb_args['name'])
        tts_proxy(sentence)
        print 'Pepper:', sentence

        counter = 0

        while counter < 3:
            speech_text = speech_proxy('')
            print userdata.kb_args['name'] + ":", speech_text.outp

            if speech_text.outp == 'Couldn\'t understand':
                sentence = random.choice(nlg.dialogue['not_understood']).format(userdata.kb_args['name'])
                tts_proxy(sentence)
                print 'Pepper:', sentence
                counter += 1

            else:
                sentence = nlg.dialogue['info_saved'][0].format(userdata.kb_args['name'])
                tts_proxy(sentence)
                print 'Pepper:', sentence

                userdata.kb_args['ID'] = speech_text.outp
                userdata.db_cursor.execute('UPDATE userdata SET id = ? WHERE name = ? AND id = ?', (speech_text.outp, userdata.kb_args['name'], ''))
                userdata.db_con.commit()

                return 'predprocess'

        userdata.status = 'not_understood'
        return 'goodbye'

class Day(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['predprocess', 'goodbye'],
                             input_keys=['kb_args', 'db_con', 'db_cursor'],
                             output_keys=['status', 'kb_args'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DAY')

        sentence = nlg.dialogue['question'][0].format(userdata.kb_args['name'])
        tts_proxy(sentence)
        print 'Pepper:', sentence

        counter = 0

        while counter < 3:
            speech_text = speech_proxy('')
            print userdata.kb_args['name'] + ":", speech_text.outp

            if ('1' in speech_text.outp) or ('one' in speech_text.outp.lower()):
                sentence = nlg.dialogue['day_rates'][0].format(userdata.kb_args['name'])
                tts_proxy(sentence)
                print 'Pepper:', sentence

                userdata.kb_args['day_rate'] = '1'
                userdata.db_cursor.execute('UPDATE userdata SET day_rate = ? WHERE name = ? AND id = ?', (1, userdata.kb_args['name'], userdata.kb_args['ID']))
                userdata.db_con.commit()
                
                return 'predprocess'

            elif ('2' in speech_text.outp) or ('two' in speech_text.outp.lower()):
                sentence = nlg.dialogue['day_rates'][1].format(userdata.kb_args['name'])
                tts_proxy(sentence)
                print 'Pepper:', sentence

                userdata.kb_args['day_rate'] = '2'
                userdata.db_cursor.execute('UPDATE userdata SET day_rate = ? WHERE name = ? AND id = ?', (2, userdata.kb_args['name'], userdata.kb_args['ID']))
                userdata.db_con.commit()
                
                return 'predprocess'

            if ('3' in speech_text.outp) or ('three' in speech_text.outp.lower()):
                sentence = nlg.dialogue['day_rates'][2].format(userdata.kb_args['name'])
                tts_proxy(sentence)
                print 'Pepper:', sentence

                userdata.kb_args['day_rate'] = '3'
                userdata.db_cursor.execute('UPDATE userdata SET day_rate = ? WHERE name = ? AND id = ?', (3, userdata.kb_args['name'], userdata.kb_args['ID']))
                userdata.db_con.commit()
                
                return 'predprocess'

            else:
                sentence = random.choice(nlg.dialogue['not_understood']).format(userdata.kb_args['name'])
                tts_proxy(sentence)
                print 'Pepper:', sentence
                counter += 1

        userdata.status = 'not_understood'
        return 'goodbye'

class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['kb_args', 'status'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DAY')

        if userdata.status == 'not_understood':
            sentence = nlg.dialogue['goodbye'][-1].format(userdata.kb_args['name'])
            tts_proxy(sentence)
            print 'Pepper:', sentence

        else:
            sentence = random.choice(nlg.dialogue['goodbye'][:-1]).format(userdata.kb_args['name'])
            tts_proxy(sentence)
            print 'Pepper:', sentence

        return 'terminate'

def main():

    try:
        con = sqlite3.connect('knowledge_base.db')
        cursor = con.cursor()

        cursor.execute('CREATE TABLE IF NOT EXISTS userdata(name TEXT, id TEXT, day_rate INTEGER)')
        con.commit()

        rospy.loginfo('Knowledge Base DB is saved to ' + os.getcwd())

        sm = smach.StateMachine(outcomes=['outcome_termination'])

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        sm.userdata.sm_status = 'understood'        # to inform states about the status ('not_understood', 'understood')
        sm.userdata.sm_kb_args = {'name': 'User',   # arguments for the knowledge base
                                  'ID': '0',
                                  'day_rate': '0'}

        sm.userdata.sm_db_con = con
        sm.userdata.sm_db_cursor = cursor

        with sm:
            smach.StateMachine.add('PREDPROCESS', PredProcess(),
                                   transitions={'hello':'HELLO',
                                                'day':'DAY',
                                                'goodbye':'GOODBYE'},
                                   remapping={'status':'sm_status',
                                              'kb_args':'sm_kb_args',
                                              'db_con':'sm_db_con',
                                              'db_cursor':'sm_db_cursor'})
            
            smach.StateMachine.add('HELLO', Hello(),
                                   transitions={'predprocess':'PREDPROCESS',
                                                'goodbye':'GOODBYE'},
                                   remapping={'status':'sm_status',
                                              'kb_args':'sm_kb_args',
                                              'db_con':'sm_db_con',
                                              'db_cursor':'sm_db_cursor'})
            
            smach.StateMachine.add('DAY', Day(),
                                   transitions={'predprocess':'PREDPROCESS',
                                                'goodbye':'GOODBYE'},
                                   remapping={'kb_args':'sm_kb_args',
                                              'status':'sm_status',
                                              'db_con':'sm_db_con',
                                              'db_cursor':'sm_db_cursor'})

            smach.StateMachine.add('GOODBYE', Goodbye(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'kb_args':'sm_kb_args',
                                              'status':'sm_status'})

        outcome = sm.execute()
        rospy.spin()
        sis.stop()

    except:
        print('Unexpected error:', sys.exc_info()[0])

    finally:
        con.close()
        

if __name__ == '__main__':
    main()
