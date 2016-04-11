import os
import glob

sources = []
includes = []
prj_path = './'

Import('genv', 'trg', 'apps', 'bsp')

# Function to find already included files in the source list
def add_sources(src2add):
    global sources
    global genv

    for tmp_src in genv.Glob(src2add):
        try:
            sources.remove(File(tmp_src))
        except ValueError:
            pass
        sources.append(File(tmp_src))

# Function to find already included files in the include list
def add_include(incpath2add):
    global includes
    global genv
    incpath = os.path.dirname(incpath2add)

    try:
        includes.remove(str(incpath))
    except ValueError:
        pass
    includes.append(str(incpath))

# Add source files from HEAD/demo/<APP_NAME>/<CONF_NAME>
def prep_demo(__dconf, __dpath):
    global prj_path
    # Add demo root files
    add_include(prj_path + 'demo/*')
    add_sources(prj_path + 'demo/*.c')
    if 'demo' in __dconf:
        add_include(__dpath + '/')
        add_sources(__dpath + '/*.c')
        for demo_file in __dconf['demo']:
            add_include(__dpath + '/' + demo_file)
            add_sources(__dpath + '/' + demo_file + '.c')

def prep_emb6(__dconf):
    global prj_path
    global genv

    emb6_path = prj_path + 'emb6/'
    emb6_modules = genv.SConscript(emb6_path + 'SConscript')

    # Add emb6 files from HEAD/emb6/ folder
    add_include(emb6_path)
    add_include(emb6_path + 'inc/')
    add_sources(emb6_path + '*.c')
    if 'emb6' in __dconf:
        for req_libmodule in __dconf['emb6']:
            if req_libmodule in emb6_modules:
                for lib_file in emb6_modules[req_libmodule]:
                    add_include(emb6_path + 'inc/' + lib_file)
                    add_sources(emb6_path + 'src/' + lib_file+'.c')

def prep_freertos(__dconf):
    global prj_path
    global genv

    freertos_path = prj_path + 'freertos/'
    freertos_modules = genv.SConscript(freertos_path + 'SConscript')

    # Add freertos files from HEAD/freertos/ folder
    add_include(freertos_path)
    add_include(freertos_path + '*.h')
    add_sources(freertos_path + '*.c')
    if 'freertos' in __dconf:
        for req_libmodule in __dconf['freertos']:
            if req_libmodule in freertos_modules:
                for lib_file in freertos_modules[req_libmodule]:
                    add_include(freertos_path + lib_file)
                    add_sources(freertos_path + lib_file + '.c')
                    print 'HALLO: ' + freertos_path + lib_file + '.c'

# Add utils headers from HEAD/utils/ folder
def prep_utils(__dconf):
    global prj_path

    utils_path = prj_path + 'utils/'
    if 'utils' in __dconf:
        for util_file in __dconf['utils']:
            add_include(utils_path + 'inc/' + util_file)
            add_sources(utils_path + 'src/' + util_file+'.c')

def prep_apps(__app, __aconf):
    global prj_path
    global genv

    if __aconf != '':
        demo_path = prj_path + 'demo/' + __app + '/' + __aconf
    else:
        demo_path = prj_path + 'demo/' + __app

    # Import configuration settings
    demo_conf = genv.SConscript(demo_path + '/SConscript')

    prep_demo(demo_conf, demo_path)

    prep_emb6(demo_conf)

    prep_freertos(demo_conf)

    prep_utils(demo_conf)

    genv.MergeFlags(demo_conf)

# Process core source and headers
def prep_board_core(__trg_path, __conf):
    bsp_path = __trg_path + 'bsp/'

    # Add board source files and headers
    add_include(bsp_path + bsp['id'] + '/')
    add_sources(bsp_path + bsp['id'] + '/*.c')
    add_include(bsp_path)
    add_sources(bsp_path + '*.c')

    # Add common source files and headers
    add_include(__trg_path)
    add_sources(__trg_path + '*.c')

# Add mcu source files and libs
def prep_board_mcu(__trg_path, __conf):
    mcu_path = __trg_path + 'mcu/'

    add_include(mcu_path + __conf['mcu']+'/')
    add_sources(mcu_path + __conf['mcu']+'/*.c')

# Add transceiver source files and libs
def prep_board_if(__trg_path,__conf) :
    global bsp
    global genv
    if_path  = __trg_path + 'if/'

    genv.MergeFlags({'CPPPATH' : [os.path.dirname(if_path + __conf['if']+'/')]})
    add_sources(if_path + __conf['if']+'/*.c')

    #Define transmitter output power macro
    mmac = 'MAC_ADDR_WORD='+ bsp['mac_addr']
    genv.MergeFlags({'CPPDEFINES' : mmac})

    #Define transmitter output power macro
    mtx_pwr = [('TX_POWER', bsp['txrx'][0])]
    genv.MergeFlags({'CPPDEFINES' : mtx_pwr})

    #Define transmitter receive sensitivity macro
    mrx_sens = 'RX_SENSITIVITY=' + bsp['txrx'][1]
    genv.MergeFlags({'CPPDEFINES' : mrx_sens})

    #Define transmitter modulation macro
    mmode = 'MODULATION='+ bsp['mode']
    genv.MergeFlags({'CPPDEFINES' : mmode})

def prep_board_arch(__trg_path,__conf):
    global bsp_path
    global genv

    arch_path = __trg_path + 'arch/'

    # Path ./target/arch/<arch>/<family>/<vendor>/
    mcu_path = arch_path + __conf['arch'] + '/' + \
               __conf['family'] + '/' + __conf['vendor']

    # Import arch configuration
    arch_conf = genv.SConscript(mcu_path+'/SConscript')

    # Add all include folder and sources which are common for a selected arch
    if 'extra' in arch_conf:
          for extra_dir in arch_conf['extra']:
              add_include(mcu_path + '/' +extra_dir+'/inc/')
              add_sources(mcu_path +'/'+ extra_dir +'/src/*.c')

    # Add all include folder and sources for a selected device
    add_include(mcu_path+'/device/'+__conf['cpu']+'/inc/')
    add_sources(mcu_path+'/device/'+__conf['cpu']+'/src/*.c')

    # Append toolchain configuration
    # Path ./target/arch/<arch>/<family>/<vendor>/SConscript
    toolchain = genv.SConscript(mcu_path+'/toolchain/'+\
                    __conf['toolchain']+'/SConscript')
    genv['CC']      = toolchain['CC']
    genv['AS']      = toolchain['AS']
    genv['LINK']    = toolchain['LINK']
    genv['OBJCOPY'] = toolchain['OBJCOPY']
    genv['SIZE']    = toolchain['SIZE']
    genv.MergeFlags(toolchain)

    # Add startup file
    if 'startupfile' in __conf:
          add_sources(mcu_path+'/device/'+__conf['cpu']+\
              '/startup/'+__conf['startupfile'])

    # Add linker script file
    if 'scriptfile' in __conf:
        lflags = '-T'+mcu_path+'/device/'+\
                 __conf['cpu']+'/ldscript/'+__conf['scriptfile']
        genv.MergeFlags({'LINKFLAGS' : lflags})

def prep_board():
    global prj_path
    global bsp
    global genv

    trg_path = prj_path + 'target/'
    board_conf = genv.SConscript(trg_path + 'bsp/' + bsp['id']+'/SConscript')

    # Merge core flags
    print trg_path + 'bsp/' + bsp['id']+'/SConscript'

    genv.MergeFlags(board_conf['std'])

    prep_board_core(trg_path, board_conf['brd'])

    prep_board_mcu(trg_path, board_conf['brd'])

    prep_board_if(trg_path, board_conf['brd'])

    prep_board_arch(trg_path, board_conf['brd'])

################################################################################
############################## MAIN APPLICATION ################################
################################################################################
add_include(prj_path)
#genv.Append(CPPPATH = ['./'])

for app_conf in apps:
    app = app_conf[0]
    conf = app_conf[1]
    print '> Configure project for ' + app + ' application' + \
          ' and ' + conf + ' configuration'
    prep_apps(app, conf)

# Import board configuration 
prep_board()

#Define logger level
try:
    Import('log_lvl')
    __log_lvl = 'LOGGER_LEVEL='+ log_lvl
    genv.MergeFlags({'CPPDEFINES' : __log_lvl})
except:
    pass

# Compile program
genv.MergeFlags({'CPPPATH' : includes})

Delete(trg+'.elf')
retf = genv.Program(target = trg  + '.elf', source = sources)
genv.Clean(retf, '*')

# Show program size
psize = genv.Command(' ', trg + '.elf', Action('$SIZE $SOURCE'))
genv.Clean(psize, '*')

# Create hex file
Delete(trg+'.hex')
hex_file = genv.Command(trg+'.hex', trg+'.elf', Action('$OBJCOPY -O ihex $SOURCE $TARGET', '$OBJCOPYCOMSTR'))
genv.Clean(hex_file, '*')

# Create binary
#Delete(trg+'.bin')
#bin_file = genv.Command(trg+'.bin', trg+'.elf', Action('$OBJCOPY -O binary $SOURCE $TARGET', '$OBJCOPYCOMSTR'))
#genv.Clean(bin_file, '*')

Return('retf')
