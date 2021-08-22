import os
from datetime import datetime

import prettytable
import textwrap

import rospy

class LogManager():
    def __init__(self, fileName):
        # gets session name from rosparam
        self.sessionName = str(rospy.get_param('session'))

        # sets log file name
        self.fileName = fileName + '.log'

        # changing directory to the file's current directory
        os.chdir(os.path.dirname(os.path.abspath(__file__)))

        # sets the path of the directory that holds session folders
        self.logDir = os.path.join(os.getcwd(), 'SessionReports')

        # gets file path
        self.file = self.setFileDir()

        # if the file is being created instead of connecting an existing open file, it initializes some information
        if self.fileName not in os.listdir(self.fileDir):
            print os.listdir(self.fileDir)
            self.initialize()

    def setFileDir(self):
        # creates main folder if not exists
        if not os.path.isdir('SessionReports'):
            os.mkdir('SessionReports')

        # if the session folder is set, use it
        if str(rospy.get_param('session_folder')) != 'NONE':
            self.fileDir = str(rospy.get_param('session_folder'))
            self.file = os.path.join(self.fileDir, self.fileName)
            return self.file

        # if there are more than 1 sessions that have same name, this number will declare the order of the session in the folder name (e.g.: Experiment(4))
        fileNumber = 0

        for f in os.listdir(self.logDir):
            # name of the folder without timestamp
            rootName = '_'.join(f.split('_')[:-1])

            # if folder has a fileNumber
            if rootName[-3] == '(' and rootName[-1] == ')' and rootName[-2].isdigit():

                if rootName[:-3] == self.sessionName:
                    fileNumber = max(fileNumber, int(rootName[-2]) + 1)

                rootName = rootName[:-3]

            if rootName == self.sessionName:

                # if the session is not closed, use it
                if not f.endswith('[CLOSED]'):
                    self.fileDir = os.path.join(self.logDir, f)
                    rospy.set_param('session_folder', self.fileDir)

                    self.file = os.path.join(self.fileDir, self.fileName)

                    return self.file

                # if the session is closed, and the fileNumber is 0.
                elif f.endswith('[CLOSED]') and fileNumber == 0:
                    fileNumber = 1

        # create sesison directory path
        if fileNumber:
            self.fileDir = os.path.join(self.logDir, self.sessionName + '(' + str(fileNumber) + ')_' + str(datetime.now()).replace(' ', '::'))
        else:
            self.fileDir = os.path.join(self.logDir, self.sessionName + '_' + str(datetime.now()).replace(' ', '::'))

        # create the session directory
        os.mkdir(self.fileDir)

        # change rosparam to the path of the session
        rospy.set_param('session_folder', self.fileDir)

        # change file path
        self.file = os.path.join(self.fileDir, self.fileName)

        return self.file

    def close(self):
        '''
        Closes the file and prevents from reusing
        '''

        self.separate(3)

        # declares that the log file is closed by adding this line to the end of the log file.
        self.write('FILE CLOSED')

        # write '[CLOSED]' at the end of the session folder's name, if not
        try:
            if not self.fileDir.endswith('[CLOSED]'):
                os.rename(self.fileDir, self.fileDir + '[CLOSED]')
        except:
            pass

        # change session path to new one
        self.fileDir = self.fileDir + '[CLOSED]'
        self.file = os.path.join(self.fileDir, self.fileName)

    def write(self, text, dateTime=True, printText=True):
        '''
        Write to file

        PARAMETERS:
        -----------
        text: Text itself to be written to file\n
        dateTime: Add dateTime at the beggining of the line\n
        printText: Print text to the terminal after writing in the file
        '''

        # if datetime is wanted to be added at the begging of the line
        if dateTime:

            # if \n is (can be more than one) put at the beggining of the text instead of the end, ...
            # ...it prevents from writing the datetime, blank lines and the text itself.
            if text[0] == '\n':
                count = 0

                for i in text:
                    if i == '\n':
                        count += 1
                    else:
                        break

                text = '\n'*count + str(datetime.now()).replace(' ', '_') + ' - ' + text[count:]
            else:
                text = str(datetime.now()).replace(' ', '_') + ' - ' + text

        # writes the text to file
        try:
            with open(self.file, 'a') as f:
                f.write(text + '\n')
        except:
            try:
                with open(os.path.join(self.fileDir + '[CLOSED]', self.fileName), 'a') as f:
                    f.write(text + '\n')
            
            except:
                pass

        # prints the text to terminal
        if printText:
            print(text + '\n')

    def separate(self, count=2):
        '''
        Writing dashes to file as a separator line

        PARAMETERS:
        -----------
        count: Number of separate lines
        '''

        for i in range(count):
            self.write('-'*50, False, False)

    def initialize(self):
        '''
        Writes the file name, package's src directory and the log files' directory
        '''

        # writing file name
        self.write('Name: ' + ''.join(self.fileName.split('.')[:-1]), False)
        self.separate(1)

        # writing the package's src file's directory
        self.write('Working directory: ' + os.getcwd())

        # writing the log files' directory
        self.write('Log directory: ' + self.logDir)
        self.separate()

    def createTable(self, cols, dateTime=True):
        '''
        Creates a table that can be added rows and printed to file

        PARAMETERS:
        -----------
        cols: Single list that contains the column values\n
        dateTime: Add datetime as a new column. It manages the row values automatically
        '''

        if dateTime:
            # adds a column to table that keeps the row's adding time
            cols.append('Time')
        
        # creates table
        self.table = prettytable.PrettyTable(cols)

        # sets horizontal lines between each row
        self.table.hrules = True

        # sets table information
        self.tableDateTime = dateTime
        self.tableState = ''
        self.tableCondition = ''

    def tableAddRow(self, row):
        '''
        Adds row to table. Input should be a list that has exactly same amount
        of values with the columns

        PARAMETERS:
        -----------
        row: Single list that contains the row values
        '''

        try:
            # textwrap is managing the lines of the table to prevent overwidth
            row = list(map(lambda x: textwrap.fill(x, 50), row))

            # first two columns are used to write the state of the state machine and...
            # ...the condition value
            row = [self.tableState, self.tableCondition] + row

            # if dateTime is set to true while creating the table
            if self.tableDateTime:
                # last column is used to write the current datetime
                row.append(str(datetime.now()).replace(' ', '_'))

            self.table.add_row(row)

        except Exception as e:
            self.write('WARNING: ' + e.message)

    def printTable(self):
        '''
        Writes table to file
        '''

        self.separate()
        self.write('STATE TRANSITIONS', False)
        self.separate(1)
        
        try:
            self.write(self.table.get_string(), False, False)
        except AttributeError as e:
            self.write('No table found.')

    def setTableState(self, state):
        '''
        Changes the state value of the table. This will be added to rows' 'state' column
        '''
        
        self.tableState = state

    def setTableCondition(self, condition):
        '''
        Changes the condition value of the table. This will be added to rows' 'condition' column
        '''

        self.tableCondition = condition

    def getLastLines(self, count, dateTime=True):
        '''
        Returns the specified amount of last lines from the file as a list. Returns 'False' if count is more than the existed lines' amount

        PARAMETERS:
        -----------
        count: Number of last lines that will be returned\n
        dateTime: Include datetime while returning the lines 
        '''

        try:
            with open(self.file, 'r') as f:
                lines = f.readlines()[-count:]

            lines = list(map(lambda x: x[:-1], lines))
                
            if not dateTime:
                # excluding datetime from the lines
                for i in range(len(lines)):
                    try:
                        lines[i] = ' '.join(lines[i].split()[2:])
                    except:
                        pass

            lines = list(filter(lambda x: x != '', lines))

            return lines

        except:
            return False

    def getTimeIntervalLines(self, start, stop, dateTime=True, lines=False):
        '''
        Returns the lines between the specified datetime interval. Returns 'False' if the time interval is invalid.

        PARAMETERS:
        -----------
        start: Starting time of the interval. It should be in the datetime.now() format\n
        stop: Ending time of the interval. It should be in the datetime.now() format\n
        dateTime: Include datetime while returning the lines
        '''

        # holds the requested lines
        interval = []

        try:
            if not lines:
                with open(self.file, 'r') as f:
                    lines = f.readlines()

                lines = list(map(lambda x: x[:-1], lines))

            for line in lines:
                try:
                    # extracting the line's datetime
                    lineTime = line.split(' ')[0]

                    # converting datetime from string to datetime format
                    lineTime = datetime.strptime(lineTime, '%Y-%m-%d_%H:%M:%S.%f')

                    # if lineTime is between the interval
                    if lineTime >= start and lineTime <= stop:
                        if not dateTime:
                            # excluding datetime from the 
                            try:
                                line = ' '.join(line.split()[2:])
                            except:
                                pass

                        interval.append(line)

                except:
                    pass

            return interval

        except:
            return False

    def getFileDir(self):
        '''
        Returns the session directory
        '''
        
        return self.fileDir