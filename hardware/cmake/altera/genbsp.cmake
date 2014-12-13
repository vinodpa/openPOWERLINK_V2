################################################################################
#
# CMake macro for generating the board support package for Altera designs
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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


MACRO(GENERATE_BSP EXAMPLE_NAME ALT_DEMO_DIR ALT_BSP_TARGET_DIR PROCESSOR_NAME TCIMEM_NAME ALT_PLK_IPCORE_REPO)

    # Set hardware directory internal paths
    SET(BSP_QUARTUS_DIR ${ALT_DEMO_DIR}/quartus)
    SET(BSP_SOF_DIR ${BSP_QUARTUS_DIR}/output_files)
    SET(BSP_SDK_DIR ${BSP_QUARTUS_DIR}/software)
    SET(ARM_HWLIB_PATH ${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib)
    SET(SPL_PATH ${OUT_PATH}/spl-${CFG_APP_CPU_NAME})
    
    FILE(GLOB QSYS_FILE_LIST RELATIVE "${BSP_QUARTUS_DIR}/" "${BSP_QUARTUS_DIR}/*.qsys")
    FOREACH (QSYS_FILE IN ITEMS ${QSYS_FILE_LIST})
        GET_FILENAME_COMPONENT(TOP_SYSTEM_NAME QSYS_FILE NAME_WE)
    ENDFOREACH()
    
    SET(BSP_SPL_SRC_DIR ${BSP_QUARTUS_DIR}/hps_isw_handoff/${TOP_SYSTEM_NAME}_${CFG_HOST_NAME})
    
    SET(BSP_SYSTEM_NAME system)
    IF(DEFINED CFG_ARM_BOOTLOADER_ENABLE AND CFG_ARM_BOOTLOADER_ENABLE)

    FILE(MAKE_DIRECTORY ${SPL_PATH})
    SET(SPL_GEN_ARGS "--preloader-settings-dir ${BSP_SPL_SRC_DIR} \
                      --settings ${SPL_PATH}/settings \
                      --type spl \
                      --set spl.boot.WATCHDOG_ENABLE false \
                      --set spl.boot.SDRAM_SCRUBBING true \
                      --set spl.debug.SKIP_SDRAM false \
                      --set spl.boot.SDRAM_SCRUB_REMAIN_REGION true "
    IF(DEFINED CFG_ARM_SEMIHOSTING_ENABLE AND CFG_ARM_SEMIHOSTING_ENABLE)
        SET (SPL_GEN_ARGS "${SPL_GEN_ARGS} --set spl.debug.SEMIHOSTING true --set spl.performance.SERIAL_SUPPORT false ")
    ELSE()
        SET (SPL_GEN_ARGS "${SPL_GEN_ARGS} --set spl.debug.SEMIHOSTING false --set spl.performance.SERIAL_SUPPORT true ")
    ENDIF()
    
    IF(DEFINED CFG_ARM_BOOT_FROM_SDCARD AND CFG_ARM_BOOT_FROM_SDCARD)
        SET (SPL_GEN_ARGS "${SPL_GEN_ARGS} --set spl.boot.BOOT_FROM_SDMMC true ")
    ELSE()
        SET (SPL_GEN_ARGS "${SPL_GEN_ARGS} --set spl.boot.BOOT_FROM_SDMMC false ")
    ENDIF()


$ bsp-create-settings.exe ${SPL_GEN_ARGS}

pushd ${SPL_PATH}

$ bsp-generate-files.exe --settings settings --bsp-dir bsp

popd
    ENDIF()

    # A hardware library source has to be used. Address macros alone has to be generated from .sopc file
    IF (DEFINED CFG_ARM_BSP_GEN AND (CFG_ARM_HAL_TYPE STREQUAL "hwlib"))
    
    # BSP has to be generated from .sopc file
    ELSE IF (DEFINED CFG_ARM_BSP_GEN AND (CFG_ARM_HAL_TYPE STREQUAL "BSP"))
    
    ENDIF()

    FILE(MAKE_DIRECTORY ${ALT_BSP_TARGET_DIR})
    CONFIGURE_FILE(${BSP_SDK_DIR}/${PROCESSOR_NAME}${BSP_SYSTEM_NAME}.mss ${ALT_BSP_TARGET_DIR}/${BSP_SYSTEM_NAME}.mss COPY_ONLY)
    CONFIGURE_FILE(${BSP_SDK_DIR}/${PROCESSOR_NAME}lscript.ld ${ALT_BSP_TARGET_DIR}/lscript.ld COPY_ONLY)
    
    CONFIGURE_FILE(${BSP_SDK_DIR}/${PROCESSOR_NAME}lscript-bootloader.ld ${ALT_BSP_TARGET_DIR}/lscript-bootloader.ld COPY_ONLY)
    ENDIF()

    IF(NOT ALT_LIBGEN STREQUAL "ALT_LIBGEN-NOTFOUND")
        ADD_CUSTOM_TARGET(
            bsp-${EXAMPLE_NAME} ALL
            DEPENDS ${ALT_BSP_TARGET_DIR}/${PROCESSOR_NAME}/lib/libxil.a
        )

        ADD_CUSTOM_COMMAND(
            DEPENDS ${BSP_SDKEXPORT_DIR}/${BSP_SYSTEM_NAME}.xml
            OUTPUT ${ALT_BSP_TARGET_DIR}/${PROCESSOR_NAME}/lib/libxil.a
            COMMAND ${ALT_LIBGEN} -hw ${BSP_SDKEXPORT_DIR}/${BSP_SYSTEM_NAME}.xml -lp ${ALT_PLK_IPCORE_REPO} -pe ${PROCESSOR_NAME} -od ${ALT_BSP_TARGET_DIR} -log libgen.log ${ALT_BSP_TARGET_DIR}/${BSP_SYSTEM_NAME}.mss
        )

        ADD_CUSTOM_TARGET(
            clean-bsp-${EXAMPLE_NAME}
            COMMAND ${CMAKE_COMMAND} -E remove_directory ${ALT_BSP_TARGET_DIR}
        )

        # Add all generated files to clean target
        SET(ADD_CLEAN_FILES ${ADD_CLEAN_FILES} ${ALT_BSP_TARGET_DIR})
    ELSE()
        MESSAGE(FATAL_ERROR "libgen was not found in system PATH or ISE installation")
    ENDIF()

ENDMACRO()
