################################################################################
#
# Altera Cyclone V ARM definitions for demo_mn_embedded application
#
# Copyright (c) 2014, Kalycito Infotech Private Limited
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holders nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

################################################################################
# Set paths
IF (CFG_KERNEL_STACK_PCP_HOSTIF_MODULE)

    SET(ALT_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${CFG_HOST_NAME}/${CFG_HOST_NAME})
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy "${CFG_HW_LIB_DIR}/linker/linker.ld" "${PROJECT_BINARY_DIR}")
    SET(LSSCRIPT ${PROJECT_BINARY_DIR}/linker.ld)
    SET(EXECUTABLE_CPU_NAME ${CFG_HOST_NAME})      # On link using host-interface the CPU name is Host

ELSE ()
    MESSAGE(FATAL_ERROR "Only CFG_KERNEL_STACK_PCP_HOSTIF_MODULE is currently implemented on Cyclone V!")
ENDIF ()

################################################################################
# Find boards support package
UNSET(ALT_LIB_BSP CACHE)
MESSAGE(STATUS "Searching for the board support package in ${ALT_BSP_DIR}")
FIND_LIBRARY(ALT_LIB_BSP NAME hal-arm
                     HINTS ${ALT_BSP_DIR}
            )

################################################################################
# Find stack library
IF (CFG_KERNEL_STACK_PCP_HOSTIF_MODULE)

    IF (${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        SET(LIB_HOSTIFLIB_NAME "hostiflib-host_d")
    ELSE ()
        SET(LIB_HOSTIFLIB_NAME "hostiflib-host")
    ENDIF ()

    UNSET(ALT_LIB_HOSTIF CACHE)
    MESSAGE(STATUS "Searching for LIBRARY ${LIB_HOSTIFLIB_NAME} in ${CFG_HW_LIB_DIR}/libhostiflib-host")
    FIND_LIBRARY(ALT_LIB_HOSTIF NAMES ${LIB_HOSTIFLIB_NAME}
                         HINTS ${CFG_HW_LIB_DIR}/libhostiflib-host
                )

ENDIF (CFG_KERNEL_STACK_PCP_HOSTIF_MODULE)

################################################################################
# Set architecture specific sources and include directories

SET (SOC_EDS_ROOT_PATH $ENV{SOCEDS_DEST_ROOT})
IF(${SOC_EDS_ROOT_PATH} STREQUAL "")
    MESSAGE(FATAL_ERROR "Run this program from the soc embedded shell!")
ENDIF()

SET(ARCH_HWLIB_PATH ${SOC_EDS_ROOT_PATH}/ip/altera/hps/altera_hps/hwlib)
SET(ARCH_SOC_TOOLS_PATH ${SOC_EDS_ROOT_PATH}/host_tools/altera/preloadergen)
SET(ARM_HWLIB_PATH ${ARCH_HWLIB_PATH})
SET(DEMO_ARCH_SOURCES
    ${DEMO_ARCHSOURCES}
    ${COMMON_SOURCE_DIR}/gpio/gpio-cyclone_arm.c
    ${COMMON_SOURCE_DIR}/lcd/lcdl-cyclone_arm.c
    ${COMMON_SOURCE_DIR}/system/system-cyclone_arm.c
   )

INCLUDE_DIRECTORIES(
                    ${ALT_BSP_DIR}/include
                    ${OPLK_ROOT_DIR}/stack/src/arch/altera_arm
                    ${COMMON_SOURCE_DIR}
                   )

################################################################################
# Set architecture specific definitions
ADD_DEFINITIONS(${ALT_HOST_CFLAGS} "-D__altera_arm__ -std=c99")

################################################################################
# Set architecture specific linker flags
SET(ARCH_LINKER_FLAGS " -T ${LSSCRIPT} -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9  -mcpu=cortex-a9 -mno-unaligned-access ")
#arm-altera-eabi-gcc -DCONFIG_HOSTIF_PCP=FALSE -DCONFIG_PCP=FALSE -D__altera_arm__=1 -DDEBUG_GLB_LVL=0 -I"C:\my-WS\OPLK\AltSoc\F\stack\include" -I"C:\my-WS\OPLK\AltSoc\F\hardware\drivers\hostinterface\include" -I"C:\my-WS\OPLK\AltSoc\F\stack\proj\generic\liboplkmnapp-hostif" -I"C:\my-WS\OPLK\AltSoc\F\hardware\drivers\hostinterface\src" -I"C:\my-WS\OPLK\AltSoc\F\objdicts\generic" -I"C:\my-WS\OPLK\AltSoc\F\objdicts\CiA302-4_MN" -I"C:\my-WS\OPLK\AltSoc\F\stack\src\arch\altera_arm" -I"C:\my-WS\OPLK\AltSoc\F\apps\common\src" -I"C:\my-WS\DS-5 Workspace\openPOWERLINK_V2_In_2_CMake\hwlib\include" -I"C:\my-WS\OPLK\AltSoc\F\hardware\lib\generic\alterac5arm\altera-c5soc\mn-dual-hostif-gpio\bsphost_0_hps_0\host_0_hps_0\include" -O2 -Ofast -g -Wall -std=gnu99 -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -fno-short-enums -mno-unaligned-access -MMD -MP -MF"apps/common/src/arp/arp.d" -MT"apps/common/src/arp/arp.d" -c -o "apps/common/src/arp/arp.o" "C:/my-WS/OPLK/AltSoc/F/apps/common/src/arp/arp.c"
#-O2 -Ofast -g -Wall -std=gnu99 -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -fno-short-enums -mno-unaligned-access -MMD -MP -MF"apps/common/src/arp/arp.d" -MT"apps/common/src/arp/arp.d"
#arm-altera-eabi-gcc -T "C:\altera\14.0\embedded\host_tools\mentor\gnu\arm\baremetal\arm-altera-eabi\lib\cortex-a9\cycloneV-dk-ram-custom.ld" -std=gnu99 -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9  -mcpu=cortex-a9 -mno-unaligned-access -o "demo_mn_embedded.axf"    $(demo_mn_embedded_axf_OBJECTS) $(demo_mn_embedded_axf_EXTERNAL_OBJECTS)  C:/my-WS/OPLK/AltSoc/F/stack/lib/generic/alterac5arm/altera-c5soc/mn-dual-hostif-gpio/liboplkmnapp-hostif.a C:/my-WS/OPLK/AltSoc/F/hardware/lib/generic/alterac5arm/altera-c5soc/mn-dual-hostif-gpio/libhostiflib-host/libhostiflib-host.a 
#arm-altera-eabi-gcc -T "C:\altera\14.0\embedded\host_tools\mentor\gnu\arm\baremetal\arm-altera-eabi\lib\cortex-a9\cycloneV-dk-ram-custom.ld" -std=gnu99 -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -o "openPOWERLINK_V2_In_2.axf"  ./stack/src/user/timer/timer-generic.o  ./stack/src/user/sdo/sdoasnd.o ./stack/src/user/sdo/sdocom-dummy.o ./stack/src/user/sdo/sdocom-std.o ./stack/src/user/sdo/sdocom.o ./stack/src/user/sdo/sdoseq.o ./stack/src/user/sdo/sdotest-com.o ./stack/src/user/sdo/sdotest-seq.o ./stack/src/user/sdo/sdoudp.o  ./stack/src/user/pdo/pdou.o ./stack/src/user/pdo/pdoucal-triplebufshm.o ./stack/src/user/pdo/pdoucal.o ./stack/src/user/pdo/pdoucalmem-hostif.o ./stack/src/user/pdo/pdoucalsync-hostif.o  ./stack/src/user/obd/obd.o ./stack/src/user/obd/obdcdc.o ./stack/src/user/obd/obdcreate.o  ./stack/src/user/nmt/identu.o ./stack/src/user/nmt/nmtcnu.o ./stack/src/user/nmt/nmtmnu.o ./stack/src/user/nmt/nmtu.o ./stack/src/user/nmt/statusu.o ./stack/src/user/nmt/syncu.o  ./stack/src/user/event/eventu.o ./stack/src/user/event/eventucal-nooshostif.o ./stack/src/user/event/eventucalintf-circbuf.o  ./stack/src/user/errhnd/errhndu.o ./stack/src/user/errhnd/errhnducal-hostif.o  ./stack/src/user/dll/dllucal-circbuf.o ./stack/src/user/dll/dllucal.o  ./stack/src/user/ctrl/ctrlu.o ./stack/src/user/ctrl/ctrlucal-hostif.o  ./stack/src/user/api/generic.o ./stack/src/user/api/processimage-cia302.o ./stack/src/user/api/processimage.o  ./stack/src/user/cfmu.o ./stack/src/user/ledu.o  ./stack/src/common/memmap/memmap-nooshostif.o  ./stack/src/common/circbuf/circbuf-nooshostif.o ./stack/src/common/circbuf/circbuffer.o  ./stack/src/common/ami/amile.o  ./stack/src/common/debugstr.o  ./stack/src/arch/altera_arm/lock-dualprocnoos.o ./stack/src/arch/altera_arm/target-arm.o ./stack/src/arch/altera_arm/target-mutex.o  ./hwlib/src/hwmgr/alt_address_space.o ./hwlib/src/hwmgr/alt_bridge_manager.o ./hwlib/src/hwmgr/alt_cache.o ./hwlib/src/hwmgr/alt_clock_manager.o ./hwlib/src/hwmgr/alt_dma.o ./hwlib/src/hwmgr/alt_dma_program.o ./hwlib/src/hwmgr/alt_fpga_manager.o ./hwlib/src/hwmgr/alt_generalpurpose_io.o ./hwlib/src/hwmgr/alt_globaltmr.o ./hwlib/src/hwmgr/alt_i2c.o ./hwlib/src/hwmgr/alt_interrupt.o ./hwlib/src/hwmgr/alt_mmu.o ./hwlib/src/hwmgr/alt_reset_manager.o ./hwlib/src/hwmgr/alt_system_manager.o ./hwlib/src/hwmgr/alt_timers.o ./hwlib/src/hwmgr/alt_watchdog.o  ./hwlib/io.o  ./hardware/drivers/hostinterface/src/hostiflib.o ./hardware/drivers/hostinterface/src/hostiflib_l.o ./hardware/drivers/hostinterface/src/hostiflibint-host.o ./hardware/drivers/hostinterface/src/hostiflibint_arm.o  ./contrib/trace/trace-printf.o  ./apps/demo_mn_embedded/src/app.o ./apps/demo_mn_embedded/src/event.o ./apps/demo_mn_embedded/src/main.o  ./apps/common/src/system/system-cyclone_arm.o  ./apps/common/src/lcd/lcd.o ./apps/common/src/lcd/lcdl-cyclone_arm.o  ./apps/common/src/gpio/gpio-cyclone_arm.o  ./apps/common/src/arp/arp.o   
#../../../apps/demo_mn_embedded/build/altera-arm/CMakeFiles/demo_mn_embedded.axf.dir/
#C:/altera/14.0/embedded/host_tools/mentor/gnu/arm/baremetal/bin/arm-altera-eabi-gcc.exe  -O3 -DNDEBUG     -std=c99 -mfloat-abi=soft -march=armv7-a -mtune=cortex-a9 -mcpu=cortex-a9 -T C:/my-WS/OPLK/AltSoc/F/apps/demo_mn_embedded/build/altera-arm/linker.ld  -o openPOWERLINK_V2_In_2.axf "CMakeFiles/demo_mn_embedded.axf.dir/src/main.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/src/app.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/src/event.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/C_/my-WS/OPLK/AltSoc/F/apps/common/src/lcd/lcd.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/C_/my-WS/OPLK/AltSoc/F/apps/common/src/arp/arp.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/C_/my-WS/OPLK/AltSoc/F/apps/common/src/gpio/gpio-cyclone_arm.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/C_/my-WS/OPLK/AltSoc/F/apps/common/src/lcd/lcdl-cyclone_arm.c.obj" "CMakeFiles/demo_mn_embedded.axf.dir/C_/my-WS/OPLK/AltSoc/F/apps/common/src/system/system-cyclone_arm.c.obj"   -o demo_mn_embedded.axf  -LC:/my-WS/OPLK/AltSoc/F/apps/demo_mn_embedded/../../hardware/lib/generic/alterac5arm/altera-c5soc/mn-dual-hostif-gpio/bsphost_0_hps_0/host_0_hps_0/lib  C:/my-WS/OPLK/AltSoc/F/stack/lib/generic/alterac5arm/altera-c5soc/mn-dual-hostif-gpio/liboplkmnapp-hostif.a C:/my-WS/OPLK/AltSoc/F/hardware/lib/generic/alterac5arm/altera-c5soc/mn-dual-hostif-gpio/bsphost_0_hps_0/host_0_hps_0/libhal-arm.a C:/my-WS/OPLK/AltSoc/F/hardware/lib/generic/alterac5arm/altera-c5soc/mn-dual-hostif-gpio/libhostiflib-host/libhostiflib-host.a
################################################################################
# Set architecture specific libraries

IF (NOT ${ALT_LIB_BSP} STREQUAL "ALT_LIB_BSP-NOTFOUND")
    SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${ALT_LIB_BSP})

    LINK_DIRECTORIES(${ALT_BSP_DIR}/lib)
ELSE ()
    MESSAGE(FATAL_ERROR "Board support package for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
ENDIF ()

IF (CFG_KERNEL_STACK_DIRECTLINK)
    IF (NOT ${ALT_LIB_OMETH} STREQUAL "ALT_LIB_OMETH-NOTFOUND")
        SET(ARCH_LIBRARIES ${ARCH_LIBRARIES} ${ALT_LIB_OMETH})
    ELSE ()
        MESSAGE(FATAL_ERROR "${LIB_OMETHLIB_NAME} for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found! Check the parameter CMAKE_BUILD_TYPE to confirm your 'Debug' or 'Release' settings")
    ENDIF ()

ELSEIF (CFG_KERNEL_STACK_PCP_HOSTIF_MODULE)

    IF (NOT ${ALT_LIB_HOSTIF} STREQUAL "ALT_LIB_HOSTIF-NOTFOUND")
        SET(ARCH_LIBRARIES ${ARCH_LIBRARIES} ${ALT_LIB_HOSTIF})
    ELSE ()
        MESSAGE(FATAL_ERROR "${LIB_HOSTIFLIB_NAME} for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found! Check the parameter CMAKE_BUILD_TYPE to confirm your 'Debug' or 'Release' settings")
    ENDIF ()

ENDIF (CFG_KERNEL_STACK_DIRECTLINK)