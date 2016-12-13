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


import os
import glob

## Clear all sources and includes
sources = []
includes = []

## set the current working directory as the project path
prjPath = './'


Import('genv', 'targetName', 'demos', 'bsp', 'net', 'osSel', 'mac', 'customCFlags')

# Function to add sources to compilation.
#
# This function adds source files to the compilation process. Therefore
# either single files can be added or templates using '*' as a wildcard
# (e.g. /path/to/sources/*.c)
#
# param     scr     Source file to add.
#
def addSources( src ):
    global sources
    global genv

    # Find all files matching the given filename or template and add it
    # to the list of sources.
    for file in genv.Glob( src ):
        try:
            sources.remove( File( file ) )
        except ValueError:
            pass
        sources.append( File( file ) )



# Function to add direcories to the include path.
#
# This function adds a given directory to the include path used for
# the compilation. Therefore the input is a specific directory. In
# case a file name is given as parameter the accoring directory of
# the file will be used.
#
# param     inc     A specific include path.
#
def addIncludePath( inc ):
    global includes

    # Get the path name from the parameter. In case this is a file
    # only the directory name will be used.
    path = os.path.dirname( inc )

    try:
        includes.remove( str( path ) )
    except ValueError:
        pass
    includes.append( str( path ) )


# Prepare an operating system for the build configuration.
#
# This function prepares the selected operation system for
# the actual build.
#
# param   osSel         The OS selection.
#
def prepareOS( osSel ):
    global prjPath
    global genv

    # If no OS selection was made we can skip
    # the following parts.
    if( osSel == "none"):
        return

    osPath = prjPath + osSel +'/'
    osModules = genv.SConscript(osPath + 'SConscript')

    # Add basic OS files and merge flags
    addIncludePath(osPath)
    addIncludePath(osPath + '*.h')
    addSources(osPath + '*.c')
    for module in osModules['base']:
        addIncludePath(osPath + module)
        addSources(osPath + module + '.c')

    # Merge Flags for OS Configuration
    genv.MergeFlags( osModules )



# Prepare the demo application for the build configuration.
#
# This function prepares the demo application for the actual build.
# Therefore the common demo files will be added to the compilation
# sources and the according common include directories wil be added.
# Afterwards the demo specific files and directories will be added.
#
# param   demoConf  The demo configuration which is used.
# param   demoPath  The path to the specific demo application.
#
def prepareDemo( demoConf, demoPath ):
    global prjPath

    # Add demo root files and include directory.
    addIncludePath(prjPath + 'demo/*')
    addSources(prjPath + 'demo/*.c')

    # Check for demo specific sources and include
    # directories.
    if 'demo' in demoConf:

        # Add demo specific root path to include direcoties and
        # add the the source files to the build.
        addIncludePath( demoPath + '/' )
        addSources( demoPath + '/*.c' )

        # Run through the paths included in the demo specific configuration
        # and add all of them to the build sources and include directories.
        for path in demoConf['demo']:
            addIncludePath( demoPath + '/' + path )
            addSources( demoPath + '/' + path + '.c' )



# Prepare the emb::6 core for the build configuration.
#
# This function prepares the emb::6 core for the build. As first
# step it adds the common source files and include directories. Then
# it checks for all the emb::6 modules that are required by the actual
# demo configuration. The paths included by those modules will be added
# to the include directories and the source list of the build.
#
# param   demoConf  The demo configuration which is used.
#
def prepareEmb6( demoConf ):
    global prjPath
    global genv

    emb6Path = prjPath + 'emb6/'
    emb6Modules, netModules = genv.SConscript( emb6Path + 'SConscript')

    # Add common include paths and source files.
    addIncludePath( emb6Path )
    addIncludePath( emb6Path + 'inc/' )
    addSources( emb6Path + '*.c' )

    # Check for emb::6 specific sources and include
    # directories.
    if 'emb6' in demoConf:

        # Run through all the modules that are specified by the demo
        # configuration and add the paths and sources.
        for module in demoConf['emb6']:
            if module in emb6Modules:
                for path in emb6Modules[module]:
                    addIncludePath( emb6Path + 'inc/' + path )
                    addSources( emb6Path + 'src/' + path +'.c' )


    # Include the network configuration
    for sources in netModules[net]['sources']:
        addIncludePath( emb6Path + 'inc/' + sources )
        addSources( emb6Path + 'src/' + sources +'.c' )

    # Merge Flags for Net Configuration
    genv.MergeFlags( netModules[net] )



# Prepare the utils for the build configuration.
#
# This function prepares the utilities for the actual build. It
# includes all files mentiond in the utility configuration.
#
# param   demoConf  The demo configuration which is used.
#
def prepareUtils( demoConf ):
    global prjPath

    utilsPath = prjPath + 'utils/'

    # Check for demo specific sources and include
    # directories.
    if 'utils' in demoConf:

        # Run through all the modules that are specified by the demo
        # configuration and add the paths and sources.
        for path in demoConf['utils']:
            addIncludePath( utilsPath + 'inc/' + path )
            addSources( utilsPath + 'src/' + path+'.c' )



# Prepare application for the build configuration.
#
# This function prepares the application for the actual build. Depending
# on its configuration the according sources and includes will be added.
#
# param   appl     The application that shall be configured.
# param   appConf  The app configuration which is used.
#
def prepareApp( app, appConf ):
    global prjPath
    global genv

    if appConf != '':
        demoPath = prjPath + 'demo/' + app + '/' + appConf
    else:
        demoPath = prjPath + 'demo/' + app

    # Import configuration settings
    demoConf = genv.SConscript(demoPath + '/SConscript')

    prepareDemo( demoConf, demoPath )
    prepareEmb6( demoConf )
    prepareUtils( demoConf )
    genv.MergeFlags( demoConf )



# Prepare the board support package for the build configuration.
#
# This function prepares the bsp for the actual build. The board
# support package consist of general source and header files that
# must all be used for al kind of builds.
#
# param   targetPath    Path to the target root directory.
#
def prepareBsp( targetPath ):
    bspPath = targetPath + 'bsp/'

    # Add general board support package source files and
    # include directories to build.
    addIncludePath( bspPath )
    addSources( bspPath + '*.c' )

    # Add board specific board support package source files and
    # include directories to build.
    addIncludePath( bspPath + bsp['id'] + '/' )
    addSources( bspPath + bsp['id'] + '/*.c' )


# Prepare the hardware abstraction layer for the build configuration.
#
# This function prepares the hal for the actual build. Each MCU
# requires its own HAL implementation which is included into the
# build using this function. Therefore it uses the 'mcu' configuration
# from the board configuration to determine the directory of the
# according HAL implementation.
#
# param   targetConf    Target configuration to get mcu from.
# param   targetPath    Path to the target root directory.
#
def prepareHal( targetConf, targetPath ):
    mcuPath = targetPath + 'mcu/'

    addIncludePath( mcuPath + targetConf['mcu']+'/' )
    addSources( mcuPath + targetConf['mcu']+'/*.c' )



# Prepare the radio interface for the build configuration.
#
# This function prepares the radio interface for the actual build. Each board
# uses a specific radio interface driver. Therefore this function uses the 'if'
# configuration from the board configuration to determine the directory of the
# according interface driver implementation.
#
# param   targetConf    Target configuration to get if from.
# param   targetPath    Path to the target root directory.
#
def prepareInterface( targetConf, targetPath ):
    ifPath  = targetPath + 'if/'

    # Add interface specific source files and include directories.
    addIncludePath(ifPath + targetConf['if'] + '/')
    addSources(ifPath + targetConf['if']+'/*.c')



# Prepare the achitecture for the build configuration.
#
# This function prepares the architecture for the actual build. Each MCU
# has an accoring architecture, such as MSP or Cortex. They usually share
# some common code basis which is configured here.
# Furthermore this function also configures the toolchain which is used
# for building the application for the specific target.
#
# param   targetConf    Target configuration to get if from.
# param   targetPath    Path to the target root directory.
# param   osSel         The OS selection.
#
def prepareArch( targetConf, targetPath, osSel ):
    global prjPath
    global genv

    # Assemble root arch path and vendor specific path
    # from configuration.
    archPath = targetPath + 'arch/'
    vendorPath = archPath + targetConf['arch'] + '/' + \
        targetConf['family'] + '/' + targetConf['vendor']

    # Import arch configuration
    archConf = genv.SConscript( vendorPath+'/SConscript' )

    # Add all include folder and sources which are common for a
    # selected architecture.
    if 'extra' in archConf:
        for extraPath in archConf['extra']:
            addIncludePath( vendorPath + '/' + extraPath + '/' )
            addIncludePath( vendorPath + '/' + extraPath + '/inc/' )
            addSources( vendorPath + '/' + extraPath + '/*.c' )
            addSources( vendorPath + '/' + extraPath + '/src/*.c' )

    # Add all include folder and sources for a selected device
    addIncludePath( vendorPath + '/device/' + targetConf['cpu'] + '/' )
    addIncludePath( vendorPath + '/device/' + targetConf['cpu'] + '/inc/' )
    addSources( vendorPath + '/device/' + targetConf['cpu'] + '/*.c')
    addSources( vendorPath + '/device/' + targetConf['cpu'] + '/src/*.c' )

    # Append toolchain configuration
    toolchain = genv.SConscript( vendorPath + '/toolchain/' + \
        targetConf['toolchain'] + '/SConscript' )

    genv['CC'] = toolchain['CC']
    genv['AS'] = toolchain['AS']
    genv['LINK'] = toolchain['LINK']
    genv['OBJCOPY'] = toolchain['OBJCOPY']
    genv['SIZE'] = toolchain['SIZE']
    genv.MergeFlags(toolchain)

    # Add startup file
    if 'startupfile' in targetConf:
        addSources( vendorPath + '/device/' + targetConf['cpu'] + \
            '/startup/' + targetConf['startupfile'] )

    # Add linker script file
    if 'scriptfile' in targetConf:
        lflags = '-T' + vendorPath + '/device/' + \
                 targetConf['cpu'] + '/ldscript/' + targetConf['scriptfile']
        genv.MergeFlags({'LINKFLAGS' : lflags} )

    # If no OS selection was made we can skip
    # the following parts.
    if( osSel == "none"):
        return

    # Check if the given OS is supported by the
    # given architecture.
    if( 'os' not in archConf ):
        print 'OS ' +  osSel + ' is not supported for this target.'
        exit(1)
    if( osSel not in archConf['os'] ):
        print 'OS ' +  osSel + ' is not supported for this target.'
        exit(1)

    # Get the OS configuration for the current
    # selected architecture.
    osConf = archConf['os'][osSel]       

    # Get the modules from the OS specific SConsfile
    osPath = prjPath + osSel +'/'
    osModules = genv.SConscript(osPath + 'SConscript')

    # Add the architecture specific source files and
    # include directories
    for source in osConf['extra']:
            addIncludePath( vendorPath + '/' + source )
            addSources( vendorPath + '/' + source + '/*.c')

    for module in osConf['source']:
        for source in osModules[module]:
            addIncludePath(osPath + source)
            addSources(osPath + source + '.c')

    # Merge Flags for OS Configuration
    genv.MergeFlags( osConf )



# Prepare the board for the build configuration.
#
# This function prepares the board for the actual build. Therefore
# it runs the according functions to prepare the BSP, HAL,
# radio interface and arcgitecture.
#
# param   targetConf    Target configuration to get if from.
# param   targetPath    Path to the target root directory.
#
def prepareBoard():
    global prjPath
    global bsp
    global genv

    targetPath = prjPath + 'target/'
    boardConf = genv.SConscript( targetPath + 'bsp/' + bsp['id'] + '/SConscript' )

    # Add common source files and headers
    addIncludePath( targetPath )
    addSources( targetPath + '*.c' )

    # Prepare bsp, hal interface and arch
    prepareBsp( targetPath )
    prepareHal( boardConf['brd'], targetPath )
    prepareInterface( boardConf['brd'], targetPath )
    prepareArch( boardConf['brd'], targetPath, osSel )

    # Merge core flags
    genv.MergeFlags( boardConf['std'] )


def prepareCFlags():

    if( mac is not None ):

        # Merge flags for the MAC address
        mmac = 'MAC_ADDR_WORD='+ mac
        genv.MergeFlags({'CPPDEFINES' : mmac})

    if( customCFlags is not None ):

        # Split the flags and merge them
        flags = customCFlags.split(',')
        genv.MergeFlags({'CPPDEFINES' : flags})


################################################################################
#                              MAIN SCRIPT                                     #
################################################################################

# Add root path to includes
addIncludePath( prjPath )


for demoConf in demos:
    demo = demoConf['demo'][0]
    conf = demoConf['demo'][1]
    print '> Configure project for ' + demo + ' demo' + \
          ' and ' + conf + ' configuration'
    # Prepare application with its configuration */
    prepareApp( demo, conf )

# Prepare operating system
prepareOS( osSel )
# Prepare board configuration
prepareBoard()
# Prepare Custom Flags
prepareCFlags()

#Define logger level
try:
    Import('log_lvl')
    __log_lvl = 'LOGGER_LEVEL='+ log_lvl
    genv.MergeFlags( {'CPPDEFINES' : __log_lvl} )
except:
    pass

# Set dependencies
genv.MergeFlags( {'CPPPATH' : includes} )

# Compile program
Delete( targetName + '.elf' )
retf = genv.Program( target = targetName  + '.elf', source = sources )
genv.Clean( retf, '*' )

# Show program size
psize = genv.Command( ' ', targetName + '.elf', Action('$SIZE $SOURCE') )
genv.Clean( psize, '*' )

# Create hex file
Delete( targetName + '.hex' )
hex_file = genv.Command( targetName +'.hex', targetName +'.elf', Action('$OBJCOPY -O ihex $SOURCE $TARGET', '$OBJCOPYCOMSTR') )
genv.Clean( hex_file, '*' )

Return('retf')
