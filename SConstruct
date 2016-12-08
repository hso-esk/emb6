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
from collections import defaultdict

AddOption('--colors',
          dest='use_colors', action='store_true',
          default=False,
          help='Enable colored build output')

def color_out(__env):
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


    compile = '%sCompiling %s==> %s$SOURCE%s' % \
    (colors['blue'], colors['purple'], colors['yellow'], colors['end'])
    
    link    = '%sLinking Program %s==> %s$TARGET%s' % \
    (colors['red'], colors['purple'], colors['yellow'], colors['end'])

    link_lib= '%sLinking Static Library %s==> %s$TARGET%s' % \
    (colors['red'], colors['purple'], colors['yellow'], colors['end'])

    # Set default values
    __env['CXXCOMSTR']  = compile
    __env['CCCOMSTR']   = compile
    __env['ARCOMSTR']   = link_lib
    __env['LINKCOMSTR'] = link    

# Print a logo and a project name

def assign_opt():
    AddOption('--target',
              dest='target', type='string',
              nargs=1, action='store', 
              metavar='TARGET_NAME',
              help='Target name lskfjlsdf')

    AddOption('--mac',
              dest='bsp_mac', type='string',
              nargs=1, action='store',
              metavar='MAC_ADDR',
              help='Two bytes of target mac address e.g. mac=0xffff')

    AddOption('--rx_sens','--rxs',
              dest='bsp_rxs', type='string',
              nargs=1, action='store', 
              metavar='RX_SENS',
              help='Specify receiver sensitivity. Target specific.')

    AddOption('--tx_pwr', '--txp',
              dest='bsp_txp', type='string',
              nargs=1, action='store', metavar='TX_POWER',
              help='Specify transmitter output power')

    AddOption('--logger', '--lo',
              dest='log_lvl', type='string',
              nargs=1, action='store', metavar='LOGGER_LEVEL',
              help='Specify level of logger [from 0 to 3]')

    AddOption('--verb',
              dest='verbose', type='int',
              nargs=1, action='store', metavar='VERBOSE',
              help='Show link commands [0 or 1]')

# Print list of supported target and exit
def print_trgs_exit(__trgs):
    print "List of supported targets:"
    print '%-25s%-20s%-20s' % ('Target name', 'MCU type', 'Transceiver type')
    for __trg_id  in __trgs:
        bsp_conf = SConscript('target/bsp/'+__trg_id['bsp']['id']+'/SConscript')
        print '%-25s%-20s%-20s' % (__trg_id['id'], bsp_conf['brd']['mcu'], bsp_conf['brd']['if'])
    exit(1)

# Process users options and generate target description
def proc_opt_get_trg(__genv,__trgs):
    __trg_id = __genv.GetOption('target')
    __trg_desc = get_descr(__trgs,__trg_id)
    if __trg_desc == 'null':
        print "Error: Specify target board. Use --target option"
        print_trgs_exit(__trgs)

    # Get Receiver sensitivity and overwrite default if required
    __bsp_rxs = __genv.GetOption('bsp_rxs')
    if __bsp_rxs:
        __trg_desc['bsp']['txrx'][1] = __bsp_rxs

    # Get transmitter output power and overwrite default if required
    __bsp_txp = __genv.GetOption('bsp_txp')
    if __bsp_txp:
        __trg_desc['bsp']['txrx'][0] = __bsp_txp

    # Get mac address and overwrite default if required
    __bsp_mac = __genv.GetOption('bsp_mac')
    if __bsp_mac:
        if len(__bsp_mac) != 6:
            print "Error: Specified last two bytes of MAC address are't OK. Should be --mac=0xFFFF"
            exit(1)
        else:
            __trg_desc['bsp']['mac_addr'] = __bsp_mac

    # Make output colorfull
    if not genv.GetOption('verbose'):
        color_out(genv)

    return __trg_desc

def get_descr(__db, __name):
    for __db_entry in __db:
        if __db_entry['id'] == __name:
            return __db_entry
    return 'null'

######################## MAIN ###################################
# Get global environment
genv = Environment(ENV = os.environ, tools=['gcc', 'gnulink'])

# Set number of jobs
num_cpu = int(os.environ.get('NUM_CPU', 4))
SetOption('num_jobs', num_cpu)
CacheDir('./build/cache')

# Install and assign available options
assign_opt()

# Get target description
target = proc_opt_get_trg(genv, SConscript('targets.scons'))

trg   = target['id'] + '_' + target['bsp']['mac_addr']
apps  = target['apps_conf']
bsp   = target['bsp']
args =  'genv trg apps bsp'

if genv.GetOption('log_lvl'):
    log_lvl = genv.GetOption('log_lvl')
    args += ' log_lvl'

build_dir = './build/'+ trg + '/'

file = genv.SConscript('SConscript', variant_dir=build_dir, duplicate=0, exports=args)

# copy bin file to ./bin directory 
file = genv.Install('./bin/', [file])
genv.Clean(file, '*')

print '====================================================================' 
print '> Select configuration parameters...'
print '> Target name =                  ' + trg
print '> Application and Config =       ' + str(apps)
print '> Board name =                   ' + bsp['id']
print '> MAC address =                  ' + bsp['mac_addr']
print '> Output power (dBm) =           ' + bsp['txrx'][0]
print '> Receiver\'s sensitivity (dBm) =' + bsp['txrx'][1]
print '> Transmitter\'s modulation =    ' + bsp['mode']
print '====================================================================' 
