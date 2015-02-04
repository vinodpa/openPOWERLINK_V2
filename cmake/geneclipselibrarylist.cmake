################################################################################
#
# CMake macro for generating a include list suitable for a SDK eclipse .cproject
# file
#
# Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
MACRO(GEN_ECLIPSE_LIBRARY_LIST LIB_LIST RES_LIB_LIST RES_LIB_PATH_LIST)

    SET(TMP_LIB_LIST "")
    FOREACH(LIB IN ITEMS ${LIB_LIST})
        STRING(REPLACE "$<$<NOT:$<CONFIG:DEBUG>>:" "" LIB ${LIB})
        GET_FILENAME_COMPONENT(LIB_PATH ${LIB} DIRECTORY)
        GET_FILENAME_COMPONENT(LIB ${LIB} NAME_WE)
        STRING(LENGTH ${LIB} STR_LEN)
        MATH(EXPR BEGIN "${STR_LEN} - 2")
        STRING(SUBSTRING ${LIB} ${BEGIN} 2 POST_FIX)
        IF (NOT ${POST_FIX} STREQUAL "_d")
            STRING(SUBSTRING ${LIB} 3 -1 LIB)
            SET(TMP_LIB_LIST "${TMP_LIB_LIST}\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"${LIB}\"/>\r")
            SET(TMP_LIB_PATH_LIST "${TMP_LIB_PATH_LIST}\t\t\t\t\t\t\t\t\t<listOptionValue builtIn=\"false\" value=\"${LIB_PATH}\"/>\r")
        ENDIF()
    ENDFOREACH()

    # Add to result list
    SET(${RES_LIB_LIST} ${TMP_LIB_LIST})
    SET(${RES_LIB_PATH_LIST} ${TMP_LIB_PATH_LIST})
ENDMACRO()