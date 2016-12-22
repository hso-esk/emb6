##
# emb6 is licensed under the 3-clause BSD license. This license gives everyone
# the right to use and distribute the code, either in binary or source code
# format, as long as the copyright license is retained in the source code.
#
# The emb6 is derived from the Contiki OS platform with the explicit approval
# from Adam Dunkels. However, emb6 is made independent from the OS through the
# removal of protothreads. In addition, APIs are made more flexible to gain
# more adaptivity during run-time.
#
# The license text is:
#
# Copyright (c) 2015,
# Hochschule Offenburg, University of Applied Sciences
# Laboratory Embedded Systems and Communications Electronics.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
# EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##

## Import the required modules.
import sys
import subprocess
import string
import re
import os
import atexit
import shutil
from collections import defaultdict

# workaround to fix 'command line is too long in Windows'
class ourSpawn:
    def ourspawn(self, sh, escape, cmd, args, env):
        newargs = ' '.join(args[1:])
        cmdline = cmd + " " + newargs
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        proc = subprocess.Popen(cmdline, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, startupinfo=startupinfo, shell = False, env = env)
        data, err = proc.communicate()
        rv = proc.wait()
        if rv:
            print "====="
            print err
            print "====="
        return rv

def setupSpawn( env ):
    if sys.platform == 'win32':
        buf = ourSpawn()
        buf.ourenv = env
        env['SPAWN'] = buf.ourspawn


## set the current working directory as the project path
prjPath = './'


# Prepare print style for compilation.
#
# Depending on the options selected, different print outputs can
# be used e.g. colored output or verbose output. This function
# initialized the according print styles.
#
# param   env           Environment to configure.
#
def createInfoPrints( env ):
    # Enable color building message output
    colors = {}
    if GetOption('use_colors'):
            colors['purple'] = '\033[95m'
            colors['blue']   = '\033[94m'
            colors['yellow'] = '\033[93m'
            colors['red']    = '\033[91m'
            colors['end']    = '\033[0m'
    else:
            colors['purple'] = ''
            colors['blue']   = ''
            colors['yellow'] = ''
            colors['red']    = ''
            colors['end']    = ''


    compileStr = '%sCompiling %s==> %s$SOURCE%s' % \
      (colors['blue'], colors['purple'], colors['yellow'], colors['end'])

    linkStr = '%sLinking Program %s==> %s$TARGET%s' % \
      (colors['red'], colors['purple'], colors['yellow'], colors['end'])

    linkLibStr = '%sLinking Static Library %s==> %s$TARGET%s' % \
      (colors['red'], colors['purple'], colors['yellow'], colors['end'])

    # Set values
    env['CXXCOMSTR']  = compileStr
    env['CCCOMSTR']   = compileStr
    env['ARCOMSTR']   = linkLibStr
    env['LINKCOMSTR'] = linkStr


# Create command line options for the script.
#
# The script accepts several command line options. This function
# initializes the according options.
#
def createOptions():


    AddOption('--clean-all',
              dest='cleanAll', action='store_true',
              default=False,
              help='Clean all targets')


    AddOption('--show-demos',
              dest='showDemos', action='store_true',
              default=False,
              help='Show a list of available demo applications')


    AddOption('--show-bsps',
              dest='showBsps', action='store_true',
              default=False,
              help='Show a list of available BSPs')


    AddOption('--colors',
              dest='use_colors', action='store_true',
              default=False,
              help='Enable colored build output')

    AddOption('--output',
              dest='outputName', type='string',
              nargs=1, action='store',
              metavar='<out.elf>',
              help='Name of the output file')

    AddOption('--demos',
              dest='demos', type='string',
              nargs=1, action='store',
              metavar='<demoa,demob>',
              help='Names of the demos to use as a comma separated list')

    AddOption('--bsp',
              dest='bsp', type='string',
              nargs=1, action='store',
              metavar='<bsp>',
              help='Name of the Board Support Package (BSP) to use')


    AddOption('--net',
              dest='net', type='choice',
              action='store',
              choices=['rpl-router', 'rpl-dagroot',],
              default='rpl-router',
              metavar='<net-role>',
              help='Network type and its role')

    AddOption('--os',
              dest='osSel', type='choice',
              action='store',
              choices=['none', 'freertos',],
              default='none',
              metavar='<osSel>',
              help='Oprating System to use')


    AddOption('--mac',
              dest='mac', type='string',
              nargs=1, action='store',
              metavar='<mac-address>',
              help='Last two bytes of target mac address e.g. mac=0xffff')


    AddOption('--ccflags',
              dest='customCFlags', type='string',
              nargs=1, action='store',
              metavar='<DEBUG,MAX=5>',
              help='Custom Compilation Flags as comma separated list ')

    AddOption('--logger',
              dest='log_lvl', type='string',
              nargs=1, action='store', metavar='<0/1/2/3>',
              help='Specify level of logger [from 0 to 3]')

    AddOption('--verbose',
              dest='verbose', action='store_true',
              default=False,
              help='Show build commands')



def helpAndExit():
  exit(0)



# Get available Demos
#
# This function retrieves all the available demos. Therfore it parses
# the according SCons file for the available demo configurations.
#
# return    The availabe demo configuration as array.
def getDemos():

    demosPath = prjPath + '/demo'

    # Get the available Demos
    demos = SConscript('SCons.demos')
    demos = sorted(demos)
    return demos


# Get Demos from Name
#
# Get specific Demos from a name. This function is used to obtain
# specific Demos e.g. from a name given by the command line options.
#
# param   name    Name of the Demos to get (comma separated).
#
# return  The specific Demos or 'None' if it does not exist.
def getDemosfromName( name ):
    # check if the given demos exist
    demos = []
    demosValid = 0;
    demosAvail = getDemos();

    if( name is None ):
        return None

    demosSplit = name.split(',')

    for demo in demosSplit:
        demosValid = 0;
        for demoAvail in demosAvail:
          if( demoAvail['id'] == demo ):
              demosValid = 1;
              demos += [demoAvail]
              break;

    if demosValid == 0:
        return None

    return demos


# Get available Board Support Packages
#
# This functions checks for available Board Support Packages. Therefore
# this function runs through all the BSP directory and reads one BSP
# per directory.
#
# return
def getBSPs():

    bspsPath = prjPath + 'target/bsp'

    # Get the available BSPs by checking all directories
    bspDirs = [b for b in os.listdir( bspsPath ) if os.path.isdir( os.path.join( bspsPath, b ) )]
    bspDirs = sorted( bspDirs )

    bsps = [];

    # Run through all BSP folders and get the configuration from the according
    # Scons file located in it.
    for bsp in bspDirs:
        if bsp == 'template':
            continue

        bspConf = SConscript( 'target/bsp/' + bsp + '/SConscript' )
        bsps += [{
            'id'  : bsp,
            'bsp' : bspConf
        }]
    return bsps


# Get BSP from Name
#
# Get a specific BSP from a name. This function is used to obtain
# a specific BSP e.g. from a name given by the command line options.
#
# param   name    Name of the BSP to get.
#
# return  The specific BSP or 'None' if it does not exist.
def getBSPfromName( name ):
    # check if the given bsp exists
    bspValid = 0;
    bspsAvail = getBSPs();

    if( name is None ):
        return None

    for bspAvail in bspsAvail:
        if( bspAvail['id'] == name ):
            bspValid = 1;
            break;

    if bspValid == 0:
        return None

    return bspAvail


# Print list of supported application and exit.
#
# This function retrieves all the available demos and prints them on
# the command line as a sorted list. Afterwards the script will be
# stopped.
def printDemosandExit():
    demos = getDemos()
    print "List of supported demos:"

    print '%-25s%-20s%-20s' % ('Demo Name', 'Demo', 'Configuration')
    print '======================================================================'
    for demo in demos:
        print '%-25s%-20s%-20s' % (demo['id'], demo['demo'][0], demo['demo'][1])
    exit(1)


# Print list of supported application and exit.
#
# This function retrieves all the available demos and prints them on
# the command line as a sorted list. Afterwards the script will be
# stopped.
def printBSPsandExit():
    bsps = getBSPs()
    print "List of supported Board Support Packages:"
    print '%-25s%-20s%-20s' % ('BSP Name', 'MCU Type', 'Transceiver Type')
    print '======================================================================'
    for bsp in bsps:
        print '%-25s%-20s%-20s' % (bsp['id'], bsp['bsp']['brd']['mcu'], bsp['bsp']['brd']['if'])
    exit(1)



################################################################################
#                              MAIN SCRIPT                                     #
################################################################################

Help("""
**** emb::6 Build Environment ****

Type: 'scons --demos=<DemoA, DemoB> --bsp=<bsp>' to build the program.
Type  'scons --show-demos' to display the demos which are availabe for build.
Type  'scons --show-bsps' to display the demos which are availabe for build.
""")

# Get global environment
genv = Environment( ENV = os.environ, tools=['gcc', 'gnulink'] )

# Build, Cache and Binaries directory
buildDir = "build"
cacheDir = buildDir + '/cache'
binDir = "bin"

# Configure build by choosing the number of cores to use for the build and
# by selecting the directory for the build cache.
numCPU = int( os.environ.get('numCPU', 4) )
SetOption( 'num_jobs', numCPU )
CacheDir( cacheDir )


# Create available command line options
createOptions()

if not genv.GetOption('help'):

    # workaround to fix 'command line is too long in Windows'
    # FIXME program size is removed when the fix is applied
    setupSpawn(genv)

    # Check if all targets shall be cleaned
    if genv.GetOption('cleanAll'):
        print 'Cleaning all targets ...'
        shutil.rmtree( buildDir, True )
        shutil.rmtree( cacheDir, True )
        shutil.rmtree( binDir, True )
        exit(0)

    # Make output colorfull
    if not genv.GetOption('verbose'):
        createInfoPrints(genv)

    # get demos and BSP configured vi athe command line
    demos = getDemosfromName( genv.GetOption('demos') )
    bsp = getBSPfromName( genv.GetOption('bsp') )
    net =  genv.GetOption('net')
    osSel =  genv.GetOption('osSel')
    mac =  genv.GetOption('mac')

    if( genv.GetOption('showDemos') ):
        printDemosandExit()
    if( genv.GetOption('showBsps')):
        printBSPsandExit()

    # Chek if demos are OK
    if( demos is None ):
        print( 'Invalid or no Demo configured.' )
        printDemosandExit()

    # Check if BSP is OK
    if( bsp is None ):
        print( 'Invalid or no Board Support Package configured.' )
        printBSPsandExit()

    # Check if Network Configuration is OK
    if( net is None ):
        print( 'Invalid network configured.' )
        exit(1)


    # Create the ouput name of the target. This is either the name
    # specified by command line or automatically assembled name.
    targetName = genv.GetOption('outputName')
    if( targetName is None ):

        #Use first demo as start name
        targetName = ''
        targetName += demos[0]['id']

        # Create the name from the following demos included
        for demoName in demos[1:]:
            targetName += '.' + demoName['id']

        # Add the network role
        targetName += '-' + net

        # Add the bsp name
        targetName += '-' + bsp['id']

        # Add the os name
        targetName += '-' + osSel

        if( mac is not None ):
          # Add the mac address
          targetName += '_' + mac

    # Forward some variables
    customCFlags = genv.GetOption('customCFlags')
    args = 'genv targetName demos bsp net osSel mac customCFlags'
    # Also formward the Log-Level if enabled
    if genv.GetOption('log_lvl'):
        log_lvl = genv.GetOption('log_lvl')
        args += ' log_lvl'

    buildDir = buildDir + '/'+ targetName + '/'
    file = genv.SConscript('SConscript', variant_dir = buildDir, duplicate = 0, exports = args)

    # copy bin file to ./bin directory
    file = genv.Install( binDir, [file] )
    genv.Clean( file, '*' )

    print '===================================================================='
    print '> Select configuration parameters...'
    print '> Target Name:                  ' + targetName
    if( demos[0]['demo'][1] == "" ):
        print '> Application and Config:       ' + demos[0]['id'] + ' [' + demos[0]['demo'][0] + ',' + demos[0]['demo'][1] + ']'
    else:
        print '> Application and Config:       ' + demos[0]['id'] + ' [' + demos[0]['demo'][0] + ']'
    for demo in demos[1:]:
        if( demo['demo'][1] == "" ):
            print '>                               ' + demo['id'] + ' [' + demo['demo'][0] + ']'
        else:
            print '>                               ' + demo['id'] + ' [' + demo['demo'][0] + ',' + demo['demo'][1] + ']'
    print '> BSP Name:                     ' + bsp['id']
    print '--------------------------------------------------------------------'
