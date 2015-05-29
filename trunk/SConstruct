import sys
import subprocess
import string
import re
import os
import atexit

# Set number of jobs
num_cpu = int(os.environ.get('NUM_CPU', 4))
SetOption('num_jobs', num_cpu)
CacheDir('./build/cache')

#TARGET_NAME = NAME
TARGET_NAME = ARGUMENTS.get('target', '')

#Parse targets file
TARGETS_FILE = ARGUMENTS.get('targetfile', 'SConsTargets');
if not os.path.isfile(TARGETS_FILE):
    print "Please provide target file (SConsTarget)"
    exit(1)
TARGETS = SConscript(TARGETS_FILE)

# Get global environment
genv = Environment(ENV = os.environ, tools=['gcc', 'gnulink'])

if genv['PLATFORM'] == 'win32':
    print "Environment: win32"
elif genv['PLATFORM'] == 'linux':
    print "Environment: linux"


# Enable color building message output
colors = {}
colors['cyan']   = '\033[96m'
colors['purple'] = '\033[95m'
colors['blue']   = '\033[94m'
colors['green']  = '\033[92m'
colors['yellow'] = '\033[93m'
colors['red']    = '\033[91m'
colors['end']    = '\033[0m'

#If the output is not a terminal, remove the colors
if not sys.stdout.isatty():
	print "Output is not terminal, coloring will be disabled"
	for key, value in colors.iteritems():
		colors[key] = ''

compile_source_message = '%sCompiling %s==> %s$SOURCE%s' % \
   (colors['blue'], colors['purple'], colors['yellow'], colors['end'])

link_program_message = '%sLinking Program %s==> %s$TARGET%s' % \
   (colors['red'], colors['purple'], colors['yellow'], colors['end'])

link_library_message = '%sLinking Static Library %s==> %s$TARGET%s' % \
   (colors['red'], colors['purple'], colors['yellow'], colors['end'])


# Set default values	
genv['PROGSUFFIX'] = '.elf'
genv['OBJSUFFIX'] = '.o'
genv['LIBPREFIX'] = 'lib'
genv['LIBSUFFIX'] = '.a'
genv['CXXCOMSTR'] = compile_source_message
genv['CCCOMSTR'] = compile_source_message
genv['ARCOMSTR'] = link_library_message
genv['LINKCOMSTR'] = link_program_message

Export('genv')

# Delete bin directory
#Execute(Delete("bin"))

#Parse targets
for TARGET in TARGETS:
	env = genv.Clone()

	__TARGET_NAME  = TARGET[0]                               #Specify build name

	if TARGET_NAME != __TARGET_NAME:                        #If we dont have given name in target array continue
		continue

	APPS_NAME   = TARGET[1]                             		#Specify application and configuration name

	BOARD_NAME  = TARGET[2]                                 #Specify board name

	MAC_ADR     = TARGET[3]                                 #Specify MAC address of the device
	TX_POWER    = TARGET[4]                                 #Specify transmit power in dBm
	RX_SENS     = TARGET[5]                                 #Specify receive sensitivity in dBm
	RXTX_MODE   = TARGET[6]

	BUILD_DIR = './build/'+ TARGET_NAME + '/'

	print '=========================================================================' 
	print '> Select configuration parameters...'
	print '> Target name =                 ' + TARGET_NAME
	print '> Application and Config =      ' + str(APPS_NAME)
	print '> Board name =                  ' + BOARD_NAME
	print '> MAC address =                 ' + MAC_ADR
	print '> Output power (dBm) =          ' + TX_POWER
	print '> Receiver\'s sensitivity (dBm) =' + RX_SENS
	print '> Transmitter\'s modulation =    ' + RXTX_MODE
	print '=========================================================================' 
	
	file = env.SConscript('SConscript', variant_dir=BUILD_DIR, duplicate=0, exports='env TARGET_NAME APPS_NAME BOARD_NAME MAC_ADR TX_POWER RX_SENS RXTX_MODE')
	
	# copy bin file to ./bin directory 
	file = env.Install('./bin/', [file])
	env.Clean(file, '*')
