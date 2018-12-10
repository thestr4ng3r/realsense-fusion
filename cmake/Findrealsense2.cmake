#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find Intel RealSense SDK to work with R200, F200 and SR300 devices.
# Once run this will define: 
#
# realsense2_FOUND
# realsense2_INCLUDE_DIR
# realsense2_LIBRARY
# realsense2_VERSION
#
#############################################################################

set(realsense2_INC_SEARCH_PATH /usr/local/include)
set(realsense2_LIB_SEARCH_PATH /usr/local/lib)
set(realsense2_DLL_SEARCH_PATH "")

if(MSVC)
  list(APPEND realsense2_INC_SEARCH_PATH "C:/librealsense2/include")

  list(APPEND realsense2_INC_SEARCH_PATH $ENV{realsense2_HOME}/include)
  list(APPEND realsense2_INC_SEARCH_PATH $ENV{realsense2_DIR}/include)
  list(APPEND realsense2_INC_SEARCH_PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0/include")

  list(APPEND realsense2_LIB_SEARCH_PATH $ENV{realsense2_HOME}/lib)
  list(APPEND realsense2_LIB_SEARCH_PATH $ENV{realsense2_DIR}/lib)
  
  if(CMAKE_CL_64)
    list(APPEND realsense2_LIB_SEARCH_PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x64")
    list(APPEND realsense2_DLL_SEARCH_PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x64")
  else()
    list(APPEND realsense2_LIB_SEARCH_PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0/lib/x86")
    list(APPEND realsense2_DLL_SEARCH_PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0/bin/x86")
  endif()
else()
  list(APPEND realsense2_INC_SEARCH_PATH /usr/include)
  list(APPEND realsense2_LIB_SEARCH_PATH /usr/lib)

  list(APPEND realsense2_INC_SEARCH_PATH $ENV{realsense2_HOME}/include)
  list(APPEND realsense2_LIB_SEARCH_PATH $ENV{realsense2_HOME}/lib)
  list(APPEND realsense2_INC_SEARCH_PATH $ENV{realsense2_DIR}/include)
  list(APPEND realsense2_LIB_SEARCH_PATH $ENV{realsense2_DIR}/lib)
endif()

find_path(realsense2_INCLUDE_DIR librealsense2/rs.hpp
  PATHS
    ${realsense2_INC_SEARCH_PATH}
)

find_library(realsense2_LIBRARY
  NAMES realsense2
  PATHS
    ${realsense2_LIB_SEARCH_PATH}
)

if(realsense2_DLL_SEARCH_PATH)
	find_file(realsense2_DLL
		realsense2.dll
		PATHS ${realsense2_DLL_SEARCH_PATH})
	if(realsense2_DLL STREQUAL realsense2_DLL-NOTFOUND)
		unset(realsense2_DLL)
	endif()
else()
	unset(realsense2_DLL)
endif()

if(realsense2_LIBRARY AND realsense2_INCLUDE_DIR)
  set(realsense2_FOUND TRUE)
  #vp_parse_header("${realsense2_INCLUDE_DIR}/librealsense2/rs.h" realsense2_VERSION_LINES RS2_API_MAJOR_VERSION RS2_API_MINOR_VERSION RS2_API_PATCH_VERSION)
  #set(realsense2_VERSION "${RS2_API_MAJOR_VERSION}.${RS2_API_MINOR_VERSION}.${RS2_API_PATCH_VERSION}")
  set(realsense2_VERSION 2.0.0)
else()
  set(realsense2_FOUND FALSE)
endif()
  
mark_as_advanced(
  realsense2_INCLUDE_DIR
  realsense2_LIBRARY
  realsense2_DLL
  realsense2_INC_SEARCH_PATH
  realsense2_LIB_SEARCH_PATH
  realsense2_DLL_SEARCH_PATH
)
