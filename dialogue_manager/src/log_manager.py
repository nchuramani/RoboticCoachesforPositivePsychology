import os
from datetime import datetime

import prettytable
import textwrap

class LogManager():
    def __init__(self, name, path=None):
        self.sessionName = name
        self.fileName = name + '_' + str(datetime.now()).replace(' ', '::') + '.log'

        # if a path is specified, it changes directory
        if path:
            os.chdir(path)
        else:
            os.chdir(os.path.dirname(os.path.abspath(__file__)))

        self.fileDir = os.path.join(os.getcwd(), 'logs')

        # checking whether the file is already created or not
        self.checkExistence()

    def checkExistence(self):
        if not os.path.isdir('logs'):
            os.mkdir('logs')

        # searches files to check whether the file exists or not
        for f in os.listdir(self.fileDir):

            # if there are a file that has a name starting with the specified file name
            # it splits beacuse there can be multiple files for a session (e.g.: 'exp1_...', 'exp1_frames_...', 'exp1_flow...', etc.)
            if '_'.join(f.split('_')[:-1]) == self.sessionName:
                with open(os.path.join(self.fileDir, f), 'r') as control:

                    # if the file is already created and closed
                    if 'FILE CLOSED' in control.readlines()[-1]:
                        raise IOError('It is a closed file that used for another experiment! Try with a different name.')
                    
                    # if the file is already created and still open
                    else:
                        self.file = os.path.join(self.fileDir, f)
                        return

        self.file = os.path.join(self.fileDir, self.fileName)

        # writing initial values to file
        self.initialize()

    def close(self):
        '''
        Closes the file and prevents from reusing
        '''

        self.separate(3)

        with open(self.file, 'a') as f:

            # writing this to declare that this file is closed and prevents rewriting later
            f.write('FILE CLOSED')

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
        with open(self.file, 'a') as f:
            f.write(text + '\n')

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

        with open(self.file, 'a') as f:
            for i in range(count):
                f.write('-'*50 + '\n')

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
        self.write('Log directory: ' + self.fileDir)
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

    def getTimeIntervalLines(self, start, stop, dateTime=True):
        '''
        Return the lines between the specified datetime interval. Returns 'False' if the time interval is invalid.

        PARAMETERS:
        -----------
        start: Starting time of the interval. It should be in the datetime.now() format\n
        stop: Ending time of the interval. It should be in the datetime.now() format\n
        dateTime: Include datetime while returning the lines
        '''

        # holds the requested lines
        interval = []

        try:
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

        

    def getLogPath(self):
        '''
        Returns the log file's path
        '''

        return os.path.joint(self.fileDir, self.fileName)

    def getLogDir(self):
        '''
        Returns the log file's directory
        '''

        return self.fileDir
