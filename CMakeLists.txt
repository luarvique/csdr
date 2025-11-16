# Copyright (c) 2021-2022 Jakob Ketterl <jakob.ketterl@gmx.de>
#
# This file is part of libcsdr.
#
# libcsdr is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# libcsdr is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with libcsdr.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required (VERSION 3.0)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_BUILD_TYPE STREQUAL None)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

project (csdr VERSION 0.18.2)
add_definitions(-DVERSION="${PROJECT_VERSION}")

enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)

include(GNUInstallDirs)

find_package(Threads REQUIRED)

include(CheckLibraryExists)
check_library_exists(m floorf "" CSDR_HAS_LIBM)

include(FindPkgConfig)
pkg_check_modules(FFTW3 REQUIRED fftw3f)

include(cmake/DetectIfunc.cmake)

pkg_check_modules(SAMPLERATE REQUIRED samplerate)

if(NOT DEFINED CSDR_GPL)
    set(CSDR_GPL true)
endif()

if(NOT DEFINED CSDR_IMA_ADPCM)
    set(CSDR_IMA_ADPCM true)
endif()

# Always using fast math
SET(CMAKE_C_FLAGS "-ffast-math -ftree-loop-vectorize")

# Add architecture-specific math flags
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    # NEHALEM enables up to SSE4.2
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpmath=sse -march=nehalem")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES ^aarch64.*)
    # AARCH64 mandates NEON
    set(HAS_NEON true)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a -Wformat=0")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES ^arm.*)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv7-a -Wformat=0")
    execute_process(COMMAND "cat" "/proc/cpuinfo" OUTPUT_VARIABLE CPUINFO)
    string(FIND ${CPUINFO} "neon" NEON_POSITION)
    if(NEON_POSITION GREATER_EQUAL 0)
        set(HAS_NEON true)
    endif()
    if (HAS_NEON)
        SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfloat-abi=hard -mfpu=neon -mvectorize-with-neon-quad")
    endif()
endif()

# Ask CLANG to vectorize math functions
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fveclib=libmvec")
endif()

# Set build-specific flags
SET(CMAKE_C_FLAGS_RELEASE "-O3")
SET(CMAKE_C_FLAGS_DEBUG "-g -O3")
SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-g -O3 -DNDEBUG")

# Make C++ flags same as C flags
SET(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(src)
