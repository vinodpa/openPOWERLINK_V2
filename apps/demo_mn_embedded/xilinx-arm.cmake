################################################################################
#
# CMake file of CiA 401 MN embedded demo application (Target is Zynq ARM Core)
#
# Copyright (c) 2014, Kalycito Infotech Pvt. Ltd.
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
IF(CFG_KERNEL_DUALPROCSHM)
    SET(XIL_BSP_DIR ${CFG_HW_LIB_DIR}/bsp${CFG_HOST_NAME}/${CFG_HOST_NAME})
    SET(XIL_DUALPROCSHM_DIR ${CFG_HW_LIB_DIR}/libdualprocshm-host)
    SET(SDFAT16_DIR ${CFG_HW_LIB_DIR}/libsdfat16-zc702)
    FILE(COPY ${CFG_HW_LIB_DIR}/bsp${CFG_HOST_NAME}/lscript.ld DESTINATION ${PROJECT_BINARY_DIR})
    SET(LSSCRIPT ${PROJECT_BINARY_DIR}/lscript.ld)
    SET(DEMO_CPU_NAME ${CFG_HOST_NAME})      # On direct link the CPU name is host
ELSE ()
    MESSAGE(FATAL_ERROR "Only CFG_KERNEL_DUALPROCSHM is currently implemented on Microblaze!")
ENDIF()

################################################################################
# Find boards support package
UNSET(XIL_LIB_BSP CACHE)
MESSAGE(STATUS "Searching for the board support package in ${XIL_BSP_DIR}")
FIND_LIBRARY(XIL_LIB_BSP NAME xil
                     HINTS ${XIL_BSP_DIR}/lib
            )

################################################################################
# Find driver dualprocshm library
IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    SET(LIB_DUALPROCSHM_NAME "dualprocshm-host_d")
ELSE()
    SET(LIB_DUALPROCSHM_NAME "dualprocshm-host")
ENDIF()

UNSET(XIL_LIB_DUALPROCSHM CACHE)
MESSAGE(STATUS "Searching for LIBRARY ${LIB_DUALPROCSHM_NAME} in ${CFG_HW_LIB_DIR}/libdualprocshm-host")
FIND_LIBRARY(XIL_LIB_DUALPROCSHM NAMES ${LIB_DUALPROCSHM_NAME}
                     HINTS ${XIL_DUALPROCSHM_DIR}
            )

################################################################################
# Find driver FAT16 SD card library
IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    SET(LIB_SDFAT16_NAME "sdfat16-zc702_d")
ELSE()
    SET(LIB_SDFAT16_NAME "sdfat16-zc702")
ENDIF()

UNSET(XIL_LIB_SDFAT16 CACHE)
MESSAGE(STATUS "Searching for LIBRARY ${LIB_SDFAT16_NAME} in ${CFG_HW_LIB_DIR}/libsdfat16-zc702")
FIND_LIBRARY(XIL_LIB_SDFAT16 NAMES ${LIB_SDFAT16_NAME}
                     HINTS ${SDFAT16_DIR}
            )

################################################################################
# Set architecture specific sources and include directories

SET(DEMO_ARCH_SOURCES
    ${DEMO_ARCHSOURCES}
    ${COMMON_SOURCE_DIR}/gpio/gpio-arm.c
    ${COMMON_SOURCE_DIR}/lcd/lcdl-null.c
   )

INCLUDE_DIRECTORIES(
                    ${XIL_BSP_DIR}/include
                    ${OPLK_ROOT_DIR}/stack/src/arch/xilinx_arm
                    ${COMMON_SOURCE_DIR}/gpio
                    ${SDFAT16_DIR}/include
                   )

################################################################################
# Set architecture specific definitions
ADD_DEFINITIONS(${XIL_CFLAGS} "-fmessage-length=0 -mcpu=${CFG_CPU_VERSION} -ffunction-sections -fdata-sections ")

################################################################################
# Set architecture specific linker flags
SET(ARCH_LINKER_FLAGS "${XIL_PLAT_ENDIAN} -mcpu=${CFG_CPU_VERSION} -Wl,-T -Wl,${LSSCRIPT} -Wl,-Map,${PROJECT_NAME}.map " )

################################################################################
# Set architecture specific libraries

IF(NOT ${XIL_LIB_BSP} STREQUAL "XIL_LIB_BSP-NOTFOUND" )
    SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${XIL_LIB_BSP})

    LINK_DIRECTORIES(${XIL_BSP_DIR}/lib)
ELSE()
    MESSAGE(FATAL_ERROR "Board support package for board ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
ENDIF()

IF(NOT ${XIL_LIB_DUALPROCSHM} STREQUAL "XIL_LIB_DUALPROCSHM-NOTFOUND" )
    SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${XIL_LIB_DUALPROCSHM})
    LINK_DIRECTORIES(${XIL_DUALPROCSHM_DIR})
ELSE()
    MESSAGE(FATAL_ERROR "Dual processor library for ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
ENDIF()

IF(NOT ${XIL_LIB_SDFAT16} STREQUAL "XIL_LIB_SDFAT16-NOTFOUND" )
    SET(ARCH_LIBRARIES ${ARCH_LIBRARIES} ${XIL_LIB_SDFAT16})
    LINK_DIRECTORIES(${SDFAT16_DIR})
ELSE()
    MESSAGE(FATAL_ERROR "SD card library library for ${CFG_DEMO_BOARD_NAME} and demo ${CFG_DEMO_NAME} not found!")
ENDIF()
#######################################################################
# Additional c library need to be nested to avoid linker errors
SET(XIL_ARM_CLIBS "-Wl,--start-group,-lxil,-lgcc,-lc,--end-group")
SET(ARCH_LIBRARIES  ${ARCH_LIBRARIES} ${XIL_ARM_CLIBS})

########################################################################
# Eclipse project files
GEN_ECLIPSE_FILE_LIST("${DEMO_SOURCES}" "" PART_ECLIPSE_FILE_LIST )
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GEN_ECLIPSE_FILE_LIST("${DEMO_ARCH_SOURCES}" "" PART_ECLIPSE_FILE_LIST)
SET(ECLIPSE_FILE_LIST "${ECLIPSE_FILE_LIST} ${PART_ECLIPSE_FILE_LIST}")

GET_PROPERTY(DEMO_INCLUDES DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
GEN_ECLIPSE_INCLUDE_LIST("${DEMO_INCLUDES}" ECLIPSE_INCLUDE_LIST)

CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appproject.in ${PROJECT_BINARY_DIR}/.project @ONLY)
CONFIGURE_FILE(${XIL_TOOLS_DIR}/eclipse/appcproject.in ${PROJECT_BINARY_DIR}/.cproject @ONLY)
