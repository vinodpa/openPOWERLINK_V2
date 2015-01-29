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

ELSEIF(CFG_KERNEL_DUALPROCSHM)

    SET(ALT_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${CFG_HOST_NAME}/${CFG_HOST_NAME})
    SET(ALT_DUALPROCSHM_DIR ${CFG_HW_LIB_DIR}/libdualprocshm-host)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy "${CFG_HW_LIB_DIR}/linker/linker.ld" "${PROJECT_BINARY_DIR}")
    SET(LSSCRIPT ${PROJECT_BINARY_DIR}/linker.ld)
    SET(EXECUTABLE_CPU_NAME ${CFG_HOST_NAME})      # On link using host-interface the CPU name is Host

ELSE ()
    MESSAGE(FATAL_ERROR "Only CFG_KERNEL_STACK_PCP_HOSTIF_MODULE and CFG_KERNEL_DUALPROCSHM is currently implemented on Cyclone V!")
ENDIF ()

################################################################################
# Find boards support package
UNSET(ALT_LIB_BSP CACHE)
MESSAGE(STATUS "Searching for the board support package in ${ALT_BSP_DIR}")
FIND_LIBRARY(ALT_LIB_BSP NAME hal
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

IF (CFG_KERNEL_DUALPROCSHM)
    IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        SET(LIB_DUALPROCSHM_NAME "dualprocshm-host_d")
    ELSE()
        SET(LIB_DUALPROCSHM_NAME "dualprocshm-host")
    ENDIF()

    UNSET(ALT_LIB_DUALPROCSHM CACHE)
    MESSAGE(STATUS "Searching for LIBRARY ${LIB_DUALPROCSHM_NAME} in ${CFG_HW_LIB_DIR}/libdualprocshm-host")
    FIND_LIBRARY(ALT_LIB_DUALPROCSHM NAMES ${LIB_DUALPROCSHM_NAME}
                         HINTS ${ALT_DUALPROCSHM_DIR}
            )
ENDIF (CFG_KERNEL_DUALPROCSHM)
################################################################################
# Set architecture specific sources and include directories

SET (SOC_EDS_ROOT_PATH $ENV{SOCEDS_DEST_ROOT})
IF(${SOC_EDS_ROOT_PATH} STREQUAL "")
    MESSAGE(FATAL_ERROR "Run this program from the soc embedded shell!")
ENDIF()

SET(ARCH_HWLIB_PATH ${SOC_EDS_ROOT_PATH}/ip/altera/hps/altera_hps/hwlib)
SET(ARCH_SOC_TOOLS_PATH ${SOC_EDS_ROOT_PATH}/host_tools/altera/preloadergen)
SET(ARM_HWLIB_PATH ${ARCH_HWLIB_PATH})
SET(ARCH_TOOLS_PATH ${OPLK_ROOT_DIR}/tools/altera-arm)
SET(DEMO_ARCH_SOURCES
    ${DEMO_ARCHSOURCES}
    ${COMMON_SOURCE_DIR}/gpio/gpio-cyclone_arm.c
    ${COMMON_SOURCE_DIR}/lcd/lcdl-cyclone_arm.c
    ${COMMON_SOURCE_DIR}/system/system-cyclone_arm.c
   )

IF(DEFINED CFG_${PROC_INST_NAME}_SEMIHOSTING_ENABLE AND CFG_${PROC_INST_NAME}_SEMIHOSTING_ENABLE)
    # No trace function is required
ELSE()
    SET(DEMO_ARCH_SOURCES ${DEMO_ARCHSOURCES} ${CONTRIB_SOURCE_DIR}/trace/trace-cyclone_arm.c)
ENDIF()

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

ELSEIF (CFG_KERNEL_DUALPROCSHM)
    IF(NOT ${ALT_LIB_DUALPROCSHM} STREQUAL "ALT_LIB_DUALPROCSHM-NOTFOUND" )
        SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${ALT_LIB_DUALPROCSHM})
        LINK_DIRECTORIES(${ALT_DUALPROCSHM_DIR})
    ELSE()
        MESSAGE(FATAL_ERROR "Dual processor library for ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
    ENDIF()

ENDIF (CFG_KERNEL_STACK_DIRECTLINK)