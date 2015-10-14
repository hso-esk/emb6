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

#Parse targets file
TARGETS_FILE = ARGUMENTS.get('targetfile', 'SConsTargets');
if not os.path.isfile(TARGETS_FILE):
    print "Error: Please provide correct target file (SConsTarget)"
    exit(1)
TARGETS = SConscript(TARGETS_FILE)

#Read all supported targets
target_list = []
for TARGET in TARGETS:
	target_list.append(TARGET[0])

#TARGET_NAME = NAME
AddOption('--target',
			dest='target',
			type='string',
			nargs=1,
			action='store',
			metavar='TARGET',
			help='Specify target name. Supported targets are target=${target_list}')
TARGET_NAME = GetOption('target')
			
AddOption('--mac',
			dest='mac',
			type='string',
			nargs=1,
			action='store',
			metavar='MAC_ADDR',
			help='Specify last two bytes of target mac address e.g. mac=0xffff')
MAC_ADDR = GetOption('mac')

AddOption('--rx_sens',
			dest='rx_sens',
			type='string',
			nargs=1,
			action='store',
			metavar='RX_SENS',
			help='Specify receiver sensitivity. Target specific.')
RX_SENS = GetOption('rx_sens')

AddOption('--tx_pwr',
			dest='tx_pwr',
			type='string',
			nargs=1,
			action='store',
			metavar='TX_POWER',
			help='Specify transmitter output power')
TX_POWER = GetOption('tx_pwr')

AddOption('--logger',
			dest='logger',
			type='string',
			nargs=1,
			action='store',
			metavar='LOGGER_LEVEL',
			help='Specify level of logger [from 0 to 3]')
LOGGER_LEVEL = GetOption('logger')

#Place Guards
if not TARGET_NAME:
	print "Error: Specify target board. Use --target option"
	exit(1)

if not TARGET_NAME or TARGET_NAME not in target_list:
	print "Error: target board {0} not in SconsTargets file. Use --help option".format(TARGET_NAME)
	exit(1)

if MAC_ADDR and len(MAC_ADDR) != 6:
	print "Error: Specified last two bytes of MAC address are't OK. Should be --mac=0xFFFF"
	exit(1)

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


#Parse targets
for TARGET in TARGETS:
	env = genv.Clone()

	__TARGET_NAME  = TARGET[0]                      #Specify build name

	if TARGET_NAME != __TARGET_NAME:                #If we dont have given name in target array continue
		continue
	args = 'env '
	args = args + 'TARGET_NAME '

	APPS_NAME   = TARGET[1]                         #Specify application and configuration name
	args = args + 'APPS_NAME '

	BOARD_NAME  = TARGET[2]         	        #Specify board name
	args = args + 'BOARD_NAME '

	if MAC_ADDR is None:
		MAC_ADDR    = TARGET[3]			#Specify MAC address of the device
	args = args + 'MAC_ADDR '

	if TX_POWER is None:
		TX_POWER    = TARGET[4]			#Specify transmit power in dBm
	args = args + 'TX_POWER '

	if RX_SENS is None:
		RX_SENS     = TARGET[5]			#Specify receive sensitivity in dBm
	args = args + 'RX_SENS '

	RXTX_MODE   = TARGET[6]
	args = args + 'RXTX_MODE '

	if LOGGER_LEVEL:
	   args = args + 'LOGGER_LEVEL'

	BUILD_DIR = './build/'+ TARGET_NAME + '/'

	TARGET_NAME = TARGET_NAME + '_' + MAC_ADDR

	file = env.SConscript('SConscript', variant_dir=BUILD_DIR, duplicate=0, exports=args)

	# copy bin file to ./bin directory 
	file = env.Install('./bin/', [file])
	env.Clean(file, '*')

print '====================================================================' 
print '> Select configuration parameters...'
print '> Target name =                  ' + TARGET_NAME
print '> Application and Config =       ' + str(APPS_NAME)
print '> Board name =                   ' + BOARD_NAME
print '> MAC address =                  ' + MAC_ADDR
print '> Output power (dBm) =           ' + TX_POWER
print '> Receiver\'s sensitivity (dBm) =' + RX_SENS
print '> Transmitter\'s modulation =    ' + RXTX_MODE
print '====================================================================' 