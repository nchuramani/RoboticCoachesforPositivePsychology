#!/usr/bin/env python
"""
Created on Mon Jan 16 12:23:10 2017

@author: vmhweigeok
"""
import roslib
import rospy
import smach
import smach_ros
import sys

# define state PREDPROCESS
class PredProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['hello', 'name', 'day', 'country', 'countryVer', 
                             'goodbye', 'terminate'],
                             input_keys=['pred_in'],
                             output_keys=['pred_args', 'pepperSays_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PREDPROCESS')
        
        if len(userdata.pred_in) > 2:
            if userdata.pred_in[0] == 'name':
                print 'module = name'
                userdata.pred_args = userdata.pred_in[1:3]
                userdata.pepperSays_id = -1
                return 'name'
            elif userdata.pred_in[0] == 'day':
                print 'module = day'
                userdata.pred_args = userdata.pred_in[1:3]
                userdata.pepperSays_id = -1
                return 'day'
            elif userdata.pred_in[0] == 'country':
                print 'module = country'
                userdata.pred_args = userdata.pred_in[1:3]
                userdata.pepperSays_id = -1
                return 'country'
            elif userdata.pred_in[0] == 'countryver':
                print 'module = countryver'
                userdata.pred_args = userdata.pred_in[1:3]
                userdata.pepperSays_id = -1
                return 'countryVer'
            else:
                print 'module is unknown'
                userdata.pred_args = ''
                userdata.pepperSays_id = 0
                return 'terminate'
        elif len(userdata.pred_in) > 1:
            if userdata.pred_in[0] == 'hello': 
                print 'module = hello'
                userdata.pred_args = userdata.pred_in[1]
                userdata.pepperSays_id = -1
                return 'hello'
            elif userdata.pred_in[0] == 'goodbye':
                print 'module = goodbye'
                userdata.pred_args = userdata.pred_in[1]
                userdata.pepperSays_id = -1
                return 'goodbye'
        else: 
            print 'pred_in not compatible'
            userdata.pred_args = ''
            userdata.pepperSays_id = 0
            return 'terminate'

# define state HELLO
class Hello(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['hello_args_in'],
                             output_keys=['hello_kb_args_out', 'hello_name', 'pepperSays_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state HELLO')
        
        ud = userdata.hello_args_in
        
        if ud.lstrip('-+').isdigit() == False:
            print 'Invalid argument'
            userdata.hello_kb_args_out = ''
            userdata.pepperSays_id = 0
        else:
            if int(ud) < 0:
                userdata.hello_kb_args_out = ''
                userdata.pepperSays_id = 1
            else:
                # look for person info at KB
                a = raw_input("[KB] Get value for person Id " + str(userdata.hello_args_in) + ": ")
                a = a.lower()
                A = a.split(',')
                
                # if KB returns data                
                if len(A) > 0:
                    print 'person found in KB'
                    userdata.hello_kb_args_out = A
                    userdata.hello_name = A[0]
                    userdata.pepperSays_id = 2
                else:
                    print 'person not found in KB'
                    userdata.hello_kb_args_out = ''
                    userdata.pepperSays_id = 1
        
        return 'terminate'

# define state NAME
class Name(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['name_args_in'],
                             output_keys=['pepperSays_id', 'name_name'])

    def execute(self, userdata):
        rospy.loginfo('Executing state NAME')
        
        if len(userdata.name_args_in) > 0:
            userdata.pepperSays_id = 3
            userdata.name_name = userdata.name_args_in[1]
            
            # save info to KB
            print 'name saved to KB'
        else:
            userdata.pepperSays_id = 4
            userdata.name_name = ''
        return 'terminate'

# define state DAY
class Day(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['day_args_in'],
                             output_keys=['pepperSays_id'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DAY')
        
        if len(userdata.day_args_in) > 0:
            if userdata.day_args_in[1].isdigit():
                if userdata.day_args_in[1] == '0':
                    userdata.pepperSays_id = 5
                elif userdata.day_args_in[1] == '1':
                    userdata.pepperSays_id = 6
                elif userdata.day_args_in[1] == '2':
                    userdata.pepperSays_id = 7
                else:
                    userdata.pepperSays_id = 8
                    # save info to KB
                    print 'day and date saved to KB'
            else:
                userdata.pepperSays_id = 8
        else:
            userdata.pepperSays_id = 0
        
        return 'terminate'

# define state COUNTRY
class Country(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['country_args_in'],
                             output_keys=['pepperSays_id', 'country_capital'])

    def execute(self, userdata):
        rospy.loginfo('Executing state COUNTRY')
        
        if len(userdata.country_args_in[1]) > 0:
            
            # get info: the capital city of country
            c = raw_input("[KB] Get capital city for country " + str(userdata.country_args_in[1]) + ": ")
            c = c.lower()
            
            if len(c) > 0:
                userdata.pepperSays_id = 10
                userdata.country_capital = c
            else:
                userdata.pepperSays_id = 11
                userdata.country_capital = ''
        else:
            userdata.pepperSays_id = 9
            
        return 'terminate'

# define state COUNTRYVER
class CountryVer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['countryVer_args_in'],
                             output_keys=['pepperSays_id', 'countryVer_capital_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state COUNTRYVER')
        
        c = ''
        if len(userdata.countryVer_args_in) > 1 and userdata.countryVer_args_in[1].isdigit():
            if int(userdata.countryVer_args_in[1]) == 0:
                c = raw_input("[KB] Get country for person id " + str(userdata.countryVer_args_in[0]) + ": ")
                c = c.lower()
                # save country to KB
                print 'country saved to KB'
                userdata.pepperSays_id = 12
            elif int(userdata.countryVer_args_in[1]) == 1:
                    userdata.pepperSays_id = 13
            else:
                    userdata.pepperSays_id = 0
        else:
            userdata.pepperSays_id = 0
            
        userdata.countryVer_capital_out = c
        return 'terminate'
        
# define state GOODBYE
class Goodbye(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['terminate'],
                             input_keys=['goodbye_args_in'],
                             output_keys=['pepperSays_id', 'goodbye_name_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GOODBYE')
        
        ud = userdata.goodbye_args_in
        userdata.goodbye_name_out = ''
        
        if ud.lstrip('-+').isdigit() == False:
            print 'Invalid argument'
            userdata.pepperSays_id = 0
        else:
            if int(ud) < 0:
                userdata.pepperSays_id = 14
            else:
                # look for person name at KB
                a = raw_input("[KB] Get name for person Id " + str(userdata.goodbye_args_in) + ": ")
                a = a.lower()
                
                if len(a) > 0:
                    userdata.pepperSays_id = 15
                    userdata.goodbye_name_out = a
                else:
                    userdata.pepperSays_id = 14
        return 'terminate'

def nlg(nlg_id, nlg_args = ''):
    # For now print this information on the terminal
    # Try to include Google TTS to speak it out: https://pypi.org/project/gTTS/

    print '[NLG SECTION HERE]'
    
    pepperSays = ''
    if nlg_id == 0:
        pepperSays = 'I don\'t understand what you\'re saying.'
    elif nlg_id == 1:
        pepperSays = 'Hello, what\'s your name?'
    elif nlg_id == 2:
        pepperSays = 'Hello, ' + nlg_args + '. Are you having a good day?'
    elif nlg_id == 3:
        pepperSays = 'That\'s a good name, ' + nlg_args + '. I will remember it. Which country are you from?'
    elif nlg_id == 4:
        pepperSays = 'I don\'t quite get that. Please repeat your name.'
    elif nlg_id == 5:
        pepperSays = 'I am so happy to hear that. I would be pleased if you can teach me something today, let\'s start!'
    elif nlg_id == 6:
        pepperSays = 'A bad day shall pass, my friend. Let\'s carry on to our task today.'
    elif nlg_id == 7:
        pepperSays = 'You must be looking forward to the summer. Let\'s do the object learning now shall we?'
    elif nlg_id == 8:
        pepperSays = 'I don\'t quite get that. How is your day? Good, bad or so lala?'
    elif nlg_id == 9:
        pepperSays = 'I don\'t quite understand that. Which country you are from again?'
    elif nlg_id == 10:
        pepperSays = 'Are you from ' + nlg_args + ', the capital city?'
    elif nlg_id == 11:
        pepperSays = 'I haven \'t heard of this country on our planet. What is your country on the Earth? I mean.'
    elif nlg_id == 12:
        pepperSays = 'Oh yes, I am the smartest robot on the floor. Let\'s start our task today.'
    elif nlg_id == 13:
        pepperSays = 'I see. I am sorry I did not get it right. Let\'s move on to the task.'
    elif nlg_id == 14:
        pepperSays = 'Goodbye, mate. Hope to see you again.'
    elif nlg_id == 15:
        pepperSays = 'Goodbye, ' + nlg_args + '. See you soon.'
    else:
        pepperSays = ''
    
    print pepperSays
    
def main():
    
    try:
        # get input 
        p = raw_input("Enter predicate: ")
        p = p.lower()
        p = p.split(',')
    
        # init ros node
        rospy.init_node('smach_state_machine')
    
        # create a sm and indicate termination outcome 
        sm = smach.StateMachine(outcomes=['outcome_termination'])

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
    
        # declare and initialize global variables 
        sm.userdata.sm_pred = p
        sm.userdata.sm_args = ''                # arguments for the state machine
        sm.userdata.sm_kb_args = ''             # arguments for the knowledge base
        sm.userdata.sm_pepperSays_id = -1       # flag for what the pepper says
        sm.userdata.sm_nlg_args = ''            # arguments for the language generation module

        # open a sm container    
        with sm:
            #add states to the container
            smach.StateMachine.add('PREDPROCESS', PredProcess(),
                                   transitions={'hello':'HELLO',
                                                'name':'NAME',
                                                'day':'DAY',
                                                'country':'COUNTRY',
                                                'countryVer':'COUNTRYVER',
                                                'goodbye':'GOODBYE',
                                                'terminate':'outcome_termination'},
                                   remapping={'pred_in':'sm_pred',
                                              'pred_args':'sm_args',
                                              'pepperSays_id':'sm_pepperSays_id'})
            
            smach.StateMachine.add('HELLO', Hello(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'hello_args_in':'sm_args',
                                              'hello_kb_args_out':'sm_kb_args',
                                              'hello_name':'sm_nlg_args',
                                              'pepperSays_id':'sm_pepperSays_id'})
            
            smach.StateMachine.add('NAME', Name(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'name_args_in':'sm_args',
                                              'pepperSays_id':'sm_pepperSays_id',
                                              'name_name':'sm_nlg_args'})
            
            smach.StateMachine.add('DAY', Day(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'day_args_in':'sm_args',
                                              'pepperSays_id':'sm_pepperSays_id'})
            
            smach.StateMachine.add('COUNTRY', Country(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'country_args_in':'sm_args',
                                              'pepperSays_id':'sm_pepperSays_id',
                                              'country_capital':'sm_nlg_args'})   
            
            smach.StateMachine.add('COUNTRYVER', CountryVer(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'countryVer_args_in':'sm_args',
                                              'countryVer_capital_out':'sm_nlg_args',
                                              'pepperSays_id':'sm_pepperSays_id'})
            
            
            smach.StateMachine.add('GOODBYE', Goodbye(),
                                   transitions={'terminate':'outcome_termination'},
                                   remapping={'goodbye_args_in':'sm_args',
                                              'pepperSays_id':'sm_pepperSays_id',
                                              'goodbye_name_out':'sm_nlg_args'})
                                              
        #execute sm
        outcome = sm.execute()

        # NLG part (stub)
        nlg(sm.userdata.sm_pepperSays_id, sm.userdata.sm_nlg_args)
        
        # wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()
    
    except:
        print("Unexpected error:", sys.exc_info()[0])

if __name__=='__main__':
    main()