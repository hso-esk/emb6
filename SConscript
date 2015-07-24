import os
import glob

sources = []
includes = []

# Function to find already included files in the source list
def add_sources(src2add):
	global sources
	global env

	for tmp_src in env.Glob(src2add):
		try:
			sources.remove(File(tmp_src))
		except ValueError:
			pass
		sources.append(File(tmp_src))
		
# Function to find already included files in the include list
def add_include(incpath2add):
	global includes
	global env

	try:
		includes.remove(str(incpath2add))
	except ValueError:
		pass
	includes.append(str(incpath2add))

Import('env', 'TARGET_NAME', 'APPS_NAME', 'BOARD_NAME', 'MAC_ADDR', 'TX_POWER', 'RX_SENS', 'RXTX_MODE')
env.Append(CPPPATH = ['./'])


################################################################################ 
################ DEMO APPLICATION CONFIGURATION ################################
################################################################################
# Add demo root files
env.MergeFlags({'CPPPATH' : [os.path.dirname('./demo/*')]})
add_sources('./demo/*.c')

for app_conf in APPS_NAME:
	app = app_conf[0]
	conf = app_conf[1]
	print '> Configure project for ' + app + ' application' + ' and ' + conf + ' configuration'
	
	# Import configuration settings 
	if conf != '':
		demo_conf = env.SConscript('./demo/' + app+'/'+conf+'/SConscript')
		# Add source files from HEAD/demo/<APP_NAME>/<CONF_NAME>
		if 'demo' in demo_conf:
			add_include(os.path.dirname('./demo/'+app+'/'+conf+'/'))
			add_sources('./demo/'+app+'/'+conf+'/*.c')
			for demo_file in demo_conf['demo']:
				add_include(os.path.dirname('./demo/'+app+'/'+conf+'/'+demo_file))
				add_sources('./demo/'+app+'/'+conf+'/'+demo_file+'.c')
	else:
		demo_conf = env.SConscript('./demo/' + app + '/SConscript')	
		# Add source files from HEAD/demo/<APP_NAME>/<CONF_NAME>
		if 'demo' in demo_conf:
			env.MergeFlags({'CPPPATH' : ['./demo/'+app+'/']})
			add_sources('./demo/'+app+'/'+'/*.c')
			for demo_file in demo_conf['demo']:
				add_include(os.path.dirname('./demo/'+app+'/'+demo_file))
				add_sources('./demo/'+app+'/'+demo_file+'.c')
			
	emb6_modules = env.SConscript('./emb6/SConscript')

	# Add emb6 files from HEAD/emb6/ folder
	env.MergeFlags({'CPPPATH' : [os.path.dirname('./emb6/inc/')]})
	if 'emb6' in demo_conf:
		env.MergeFlags({'CPPPATH' : ['./emb6']})
		for req_libmodule in demo_conf['emb6']:
			if req_libmodule in emb6_modules:
				for lib_file in emb6_modules[req_libmodule]:
					add_include(os.path.dirname('./emb6/inc/' + lib_file))
					add_sources('./emb6/src/'+lib_file+'.c')

	# Add utils headers from HEAD/utils/ folder
	if 'utils' in demo_conf:
		for util_file in demo_conf['utils']:
			add_include(os.path.dirname('./utils/inc/' + util_file))
			add_sources('./utils/src/'+util_file+'.c')

	# Add defines
	if 'defines' in demo_conf:
		env.MergeFlags({'CPPDEFINES' : demo_conf['defines']})
	
	# Add gcc flags
	if 'cflags' in demo_conf:
		env.MergeFlags({'CFLAGS' : demo_conf['cflags']})
	
	# Add linker flags
	if 'ldflags' in demo_conf:
		env.MergeFlags({'LINKFLAGS' : demo_conf['ldflags']})
		
################################################################################ 
#################### TARGET BOARD CONFIGURATION ################################
################################################################################

# Import board configuration 
board_conf = env.SConscript('./target/bsp/'+BOARD_NAME+'/SConscript')

########################## ARCH SECTION ########################################

# Form a path to mcu folder
# Path ./target/arch/<arch>/<family>/<vendor>/
mcu_path = './target/arch/'+board_conf['mcu_arch']+'/'+board_conf['mcu_family']+'/'+board_conf['mcu_vendor']

# Import arch configuration 
arch_conf = env.SConscript(mcu_path+'/SConscript')

# Add all include folder and sources which are common for a selected arch
if 'extra' in arch_conf:
	for extra_dir in arch_conf['extra']:
		add_include(os.path.dirname(mcu_path + '/' +extra_dir+'/inc/'))
		add_sources(mcu_path +'/'+ extra_dir +'/src/*.c')

# Add all include folder and sources for a selected device
add_include(os.path.dirname(mcu_path+'/device/'+board_conf['mcu_cpu']+'/inc/'))
add_sources(mcu_path+'/device/'+board_conf['mcu_cpu']+'/src/*.c')

# Append toolchain configuration
# Path ./target/arch/<arch>/<family>/<vendor>/SConscript
toolchain = env.SConscript(mcu_path+'/toolchain/'+board_conf['mcu_toolchain']+'/SConscript')
env['CC'] = toolchain['CC']
env['AS'] = toolchain['AS']
env['LINK'] = toolchain['LINK']
env['OBJCOPY'] = toolchain['OBJCOPY']
env['SIZE'] = toolchain['SIZE']
env.MergeFlags(toolchain)

# Add startup file
if 'startupfile' in board_conf:
	add_sources(mcu_path+'/device/'+board_conf['mcu_cpu']+'/startup/'+board_conf['startupfile'])

# Add linker script file
if 'scriptfile' in board_conf:
	env.MergeFlags({'LINKFLAGS' : '-T'+mcu_path+'/device/'+board_conf['mcu_cpu']+'/ldscript/'+board_conf['scriptfile']})

########################## BOARD SECTION #######################################
# Add board source files and headers
add_include(os.path.dirname('./target/bsp/'+BOARD_NAME+'/'))
add_sources('./target/bsp/'+BOARD_NAME+'/*.c')
add_include(os.path.dirname('./target/bsp/'))
add_sources('./target/bsp/*.c')

# Add common source files and headers
add_include(os.path.dirname('./target/'))
add_sources('./target/*.c')

# Add mcu source files and libs
add_include(os.path.dirname('./target/mcu/'+board_conf['mcu']+'/'))
add_sources('./target/mcu/'+board_conf['mcu']+'/*.c')

# Add transceiver source files and libs
env.MergeFlags({'CPPPATH' : [os.path.dirname('./target/if/'+board_conf['if']+'/')]})
add_sources('./target/if/'+board_conf['if']+'/*.c')

#Define transmitter output power
if MAC_ADDR != '':
	MAC_ADDR = 'MAC_ADDR_WORD='+ MAC_ADDR
	env.MergeFlags({'CPPDEFINES' : MAC_ADDR})

#Define transmitter output power
if TX_POWER != '':
	tx_pwr = [('TX_POWER', TX_POWER)]
	env.MergeFlags({'CPPDEFINES' : tx_pwr})

#Define transmitter receive sensitivity
if RX_SENS != '':
	rx_sens = 'RX_SENSITIVITY='+ RX_SENS
	env.MergeFlags({'CPPDEFINES' : rx_sens})
	
#Define transmitter modulation
if RXTX_MODE != '':
	rxtx_mode = 'MODULATION='+ RXTX_MODE
	env.MergeFlags({'CPPDEFINES' : rxtx_mode})

# Add defines
if 'defines' in board_conf:
	env.MergeFlags({'CPPDEFINES' : board_conf['defines']})
	
# Add gcc flags
if 'cflags' in board_conf:
	env.MergeFlags({'CFLAGS' : board_conf['cflags']})
	
# Add linker flags
if 'ldflags' in board_conf:
	env.MergeFlags({'LINKFLAGS' : board_conf['ldflags']})
	
################################################################################ 
####################### Final Compilation ######################################
################################################################################

# Add library source files and headers
add_include(os.path.dirname('./emb6/'))
add_sources('./emb6/*.c')

# Compile program
env.MergeFlags({'CPPPATH' : includes})
Delete(TARGET_NAME+'.elf')
retf = env.Program(target = TARGET_NAME+'.elf', source = sources)
env.Clean(retf, '*')

# Show program size
psize = env.Command(' ', TARGET_NAME + '.elf', Action('$SIZE $SOURCE'))
env.Clean(psize, '*')

# Create hex file
Delete(TARGET_NAME+'.hex')
hex_file = env.Command(TARGET_NAME+'.hex', TARGET_NAME+'.elf', Action('$OBJCOPY -O ihex $SOURCE $TARGET', '$OBJCOPYCOMSTR'))
env.Clean(hex_file, '*')

# Create binary
Delete(TARGET_NAME+'.bin')
bin_file = env.Command(TARGET_NAME+'.bin', TARGET_NAME+'.elf', Action('$OBJCOPY -O binary $SOURCE $TARGET', '$OBJCOPYCOMSTR'))
env.Clean(bin_file, '*')

Return('bin_file')
