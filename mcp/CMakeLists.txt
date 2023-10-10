# TODO(evanmayer) remove this. I'm not sure it serves a useful purpose anymore.
# Helper function to add preprocesor definition of FILE_BASENAME
# to pass the filename without directory path for debugging use.
#
# Example:
#
#   define_file_basename_for_sources(my_target)
#
# Will add -DFILE_BASENAME="filename" for each source file depended on
# by my_target, where filename is the name of the file.
#
function(define_file_basename_for_sources targetname)
    get_target_property(source_files "${targetname}" SOURCES)
    foreach(sourcefile ${source_files})
        # Get source file's current list of compile definitions.
        get_property(defs SOURCE "${sourcefile}"
            PROPERTY COMPILE_DEFINITIONS)
        # Add the FILE_BASENAME=filename compile definition to the list.
        get_filename_component(basename "${sourcefile}" NAME)
        list(APPEND defs "__FILENAME__=\"${basename}\"")
        # Set the updated compile definitions on the source file.
        set_property(
            SOURCE "${sourcefile}"
            PROPERTY COMPILE_DEFINITIONS ${defs})
    endforeach()
endfunction()

# Have each target run its own style check on recompile.
#
# Note
# ----
# Optional patterns (regular expressions) can be provided to specify files
# for which linting should be skipped (e.g. files from other projects which
# were included as-is to the source base).
function(check_style targetname)
    # Find all sources (headers and source files) that contribute to the target.
    get_target_property(all_files "${targetname}" SOURCES)
    set(check_files ${all_files})

    # Filter out unwanted files using provided patterns if any.
    set(skip_patterns ${ARGN})
    list(LENGTH skip_patterns pattern_count)
    if (${pattern_count} GREATER 0)
        foreach(patt ${skip_patterns})
            list(FILTER check_files EXCLUDE REGEX ${patt})
        endforeach()

        list(LENGTH all_files all_count)
        list(LENGTH check_files check_count)
        if (${check_count} LESS ${all_count})
            list(JOIN skip_patterns ", " pattern_repr)
            message(VERBOSE
                    "In target '${targetname}', the code style for files that "
                    "match patterns ${pattern_repr} will NOT be checked.")
        endif()
    endif()

    # Run the linter.
    add_custom_command(TARGET ${targetname} PRE_BUILD
        COMMAND ${PROJECT_SOURCE_DIR}/testing/clint.sh ${check_files}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Checking Code Style in ${targetname}"
        VERBATIM
    )
endfunction()

project (blast_mcp)

cmake_minimum_required (VERSION 2.8.8)

# *****************************************************************************
# PROJECT DEFINITIONS
# *****************************************************************************
option(ENABLE_TESTING "build unit tests" ON)
option(ENABLE_TESTS_ON_BUILD "run tests during build" ON)
option(ENABLE_STYLE_CHECK "Run Google cpplint" ON)
option(ENABLE_COVERAGE "Run gcov" OFF)

option(NO_KIDS_TEST "do not write kids data or call roach fns" OFF)
if (NO_KIDS_TEST)
    add_definitions(-DNO_KIDS_TEST)
endif(NO_KIDS_TEST)

SET (CMAKE_C_FLAGS                  "-Wall -std=gnu99 -mcx16")
SET (CMAKE_C_FLAGS_DEBUG            "-O0 -fno-omit-frame-pointer -ggdb -g3")
SET (CMAKE_C_FLAGS_MINSIZEREL       "-Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE          "-O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO   "-O2 -g")

SET (CMAKE_CXX_FLAGS                "-Wall")
SET (CMAKE_CXX_FLAGS_DEBUG          "-g3")
SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

# Do a bad thing: mcp originally compiled under Debian 8. Under Ubuntu 20.04,
# the linker is less permissive of transitive dependencies. Revert it to the
# previous state to fix some linker errors raised when objects link against
# libphenom and can't get openssl symbols.
# Bkgd.: https://wiki.debian.org/ToolChain/DSOLinking#Unresolved_symbols_in_shared_libraries
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
# to atone for this sin, put some additional guardrails in place
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-undefined,--no-allow-shlib-undefined")

# Set global definitions
add_definitions(-D_GNU_SOURCE)

if (ENABLE_COVERAGE)
    # Append gcov flags
    SET(CMAKE_C_FLAGS
        "${CMAKE_C_FLAGS}\
        -fprofile-arcs\
        -ftest-coverage\
        -fno-inline-small-functions\
        -fkeep-inline-functions\
        -fkeep-static-functions"
    )
    SET(CMAKE_C_FLAGS_DEBUG
        "${CMAKE_C_FLAGS_DEBUG}\
        -fprofile-arcs\
        -ftest-coverage\
        -fno-inline-small-functions\
        -fkeep-inline-functions\
        -fkeep-static-functions"
    )
    # Clean gcov files
    file(GLOB_RECURSE GCOV_DATA ${PROJECT_BINARY_DIR}/*.gcda)
    file(GLOB_RECURSE GCOV_NOTES ${PROJECT_BINARY_DIR}/*.gcno)
    SET(GCOV_FILES ${GCOV_DATA})
    SET(GCOV_FILES ${GCOV_FILES} ${GCOV_NOTES})
    if (GCOV_FILES)
        message("Code coverage: Removing old gcov files")
        file(REMOVE ${GCOV_FILES})
    endif()
endif()

# *****************************************************************************
# EXTERNAL DEPENDENCIES
# *****************************************************************************
# Find external libs
find_package(PkgConfig)

# Find Threading lib
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

# Find ExternalProject module
include(ExternalProject)

# Used to check contents of header files
include(CheckLibraryExists)

# Do NOT enable building unit tests if you use "vanilla" debian 'jessie'.
#
# Building unit tests requires cmake>=3.5.0 to build cmocka's>=1.1.5 necessary
# to run the tests. 'jessie' comes with cmake=3.0.2 and cmocka=0.4.1 which API
# is incompatible with the newer version.
#
# If you want to build the tests, install cmake 3.5.0 from 'jessie-backports'
# repository. Follow the link below for more information:
#
#   https://www.lucas-nussbaum.net/blog/?p=947
#
# tl;dr Add these line to your /etc/apt/sources.list
#
#   deb http://archive.debian.org/debian/ jessie-backports main contrib non-free
#
# and run the commands below afterwards
#
#   sudo sh -c "echo 'Acquire::Check-Valid-Until no;' > /etc/apt/apt.conf.d/99no-check-valid-until"
#   sudo apt update
if (ENABLE_TESTING)
    include(ExternalProject)
    ExternalProject_Add(libcmocka
        GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/../external_libs/cmocka
        GIT_TAG cmocka-1.1.5
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_BUILD_TYPE:STRING=Release
    )
    ExternalProject_Get_property(libcmocka INSTALL_DIR)
    set(CMOCKA_INCLUDE_DIR ${INSTALL_DIR}/include)
    set(CMOCKA_LIBRARY ${INSTALL_DIR}/lib/libcmocka.so)
    # Allow find_package() to work
    set(cmocka_DIR ${INSTALL_DIR}/lib/cmake/cmocka)
    enable_testing()
endif(ENABLE_TESTING)

# Find gsl
pkg_check_modules(PC_GSL QUIET gsl)
set(GSL_DEFINITIONS ${PC_GSL_CFLAGS_OTHER})
find_path(GSL_INCLUDE_DIR gsl/gsl_blas.h
          HINTS ${PC_GSL_INCLUDEDIR} ${PC_GSL_INCLUDE_DIRS}
          PATH_SUFFIXES gsl REQUIRED)
find_library(GSL_LIBRARY NAMES gsl libgsl
            HINTS ${PC_GSL_LIBDIR} ${PC_GSL_LIBRARY_DIRS} REQUIRED)
find_library(GSL_CBLAS_LIBRARY NAMES gslcblas libgslcblas
            HINTS ${PC_GSL_LIBDIR} ${PC_GSL_LIBRARY_DIRS} REQUIRED)
IF(NOT GSL_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find gsl_blas.h.  Is libgsl0-dev installed?")
ENDIF()
IF(NOT GSL_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libgsl.  Is libgsl0ldb installed?")
ENDIF()

# Find glib
pkg_check_modules(PC_GLIB  glib-2.0)
set(GLIB_DEFINITIONS ${PC_GLIB_CFLAGS_OTHER})
find_path(GLIB_INCLUDE_DIR NAMES glib.h
          HINTS ${PC_GLIB_INCLUDEDIR} ${PC_GLIB_INCLUDE_DIRS}
          PATH_SUFFIXES glib-2.0 REQUIRED)
find_path(GLIBCONFIG_INCLUDE_DIR NAMES glibconfig.h
          HINTS ${PC_GLIB_INCLUDEDIR} ${PC_GLIB_INCLUDE_DIRS}
          PATH_SUFFIXES glib-2.0 REQUIRED)
find_library(GLIB_LIBRARY NAMES glib-2.0 libglib-2.0
            HINTS ${PC_GLIB_LIBDIR} ${PC_GLIB_LIBRARY_DIRS} REQUIRED)
IF(NOT GLIB_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find glib.h.  Is libglib2.0-dev installed?")
ENDIF()
IF(NOT GLIB_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libglib-2.0.  Is libglib2.0-0 installed?")
ENDIF()

# Find sensors
pkg_check_modules(PC_SENSORS QUIET sensors)
set(SENSORS_DEFINITIONS ${PC_SENSORS_CFLAGS_OTHER})
find_path(SENSORS_INCLUDE_DIR NAMES sensors.h
          HINTS ${PC_SENSORS_INCLUDEDIR} ${PC_SENSORS_INCLUDE_DIRS}
          PATH_SUFFIXES sensors REQUIRED)
find_library(SENSORS_LIBRARY NAMES sensors libsensors
            HINTS ${PC_SENSORS_LIBDIR} ${PC_SENSORS_LIBRARY_DIRS} REQUIRED)
IF(NOT SENSORS_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find sensors.h.  Is libsensors4-dev installed?")
ENDIF()
IF(NOT SENSORS_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libsensors.  Is libsensors4 installed?")
ENDIF()

# Find mosquitto
pkg_check_modules(PC_MOSQ QUIET mosquitto)
set(MOSQ_DEFINITIONS ${PC_MOSQ_CFLAGS_OTHER})
find_path(MOSQ_INCLUDE_DIR NAMES mosquitto.h
          HINTS ${PC_MOSQ_INCLUDEDIR} ${PC_MOSQ_INCLUDE_DIRS}
          PATH_SUFFIXES mosquitto REQUIRED)
find_library(MOSQ_LIBRARY NAMES mosquitto libmosquitto
            HINTS ${PC_MOSQ_LIBDIR} ${PC_MOSQ_LIBRARY_DIRS} REQUIRED)
IF(NOT MOSQ_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find mosquitto.h.  Is libmosquitto-dev installed?")
ENDIF()
IF(NOT MOSQ_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libmosquitto.  Is libmosquitto1 installed?")
ENDIF()

# Find Concurrency Kit
pkg_check_modules(PC_CK QUIET ck)
set(CK_DEFINITIONS ${PC_CK_CFLAGS_OTHER})
find_path(CK_INCLUDE_DIR NAMES ck_cc.h
          HINTS ${PC_CK_INCLUDEDIR} ${PC_CK_INCLUDE_DIRS})
find_library(CK_LIBRARY NAMES ck libck
            HINTS ${PC_CK_LIBDIR} ${PC_CK_LIBRARY_DIRS} REQUIRED)
IF(NOT CK_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find ck_cc.  Is libck-dev installed?")
ENDIF()
IF(NOT CK_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libck.  Is the libck package installed?")
ENDIF()

# add soem library
set(ECAT_LIBRARY ${PROJECT_BINARY_DIR}/soem/libethercat.a)
ExternalProject_Add(soem
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/soem
    BINARY_DIR ${PROJECT_BINARY_DIR}/soem
    URL ${PROJECT_SOURCE_DIR}/../external_libs/soem
    INSTALL_COMMAND "")

# add our version of libphenom
ExternalProject_Add(phenomproj
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/phenom
    UPDATE_COMMAND git pull
    PATCH_COMMAND ./autogen.sh
    CONFIGURE_COMMAND ./configure -q --disable-shared --enable-silent-rules
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/../external_libs/libphenom
    BUILD_IN_SOURCE 1
    BUILD_COMMAND make libphenom.la
    INSTALL_COMMAND "")
set(PH_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/phenom/src/phenomproj/include/phenom)
set(PH_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/phenom/src/phenomproj/.libs/libphenom.a)

# Find libmodbus
pkg_check_modules(PC_MODBUS QUIET libmodbus)
set(MODBUS_DEFINITIONS ${PC_MODBUS_CFLAGS_OTHER})
find_path(MODBUS_INCLUDE_DIR modbus.h
          HINTS ${PC_MODBUS_INCLUDEDIR} ${PC_MODBUS_INCLUDE_DIRS}
          REQUIRED)
find_library(MODBUS_LIBRARY NAMES libmodbus modbus
            HINTS ${PC_MODBUS_LIBDIR} ${PC_MODBUS_LIBRARY_DIRS} REQUIRED)
IF(NOT MODBUS_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find modbus.h.  Is libmodbus-dev installed?")
ENDIF()
IF(NOT MODBUS_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libmodbus.  Please install this using apt-get.")
ENDIF()

# Find libusb
pkg_check_modules(PC_USB QUIET libusb-1.0)
set(USB_DEFINITIONS ${PC_USB_CFLAGS_OTHER})
find_path(USB_INCLUDE_DIR libusb-1.0/libusb.h
          HINTS ${PC_USB_INCLUDEDIR} ${PC_USB_INCLUDE_DIRS}
          REQUIRED)
find_library(USB_LIBRARY NAMES usb-1.0 libusb-1.0
            HINTS ${PC_USB_LIBDIR} ${PC_USB_LIBRARY_DIRS} REQUIRED)
IF(NOT USB_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find libusb.h.  Is libusb-1.0-0-dev installed?")
ENDIF()
IF(NOT USB_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libusb-1.0.  Please install this using apt-get.")
ENDIF()


# *****************************************************************************
# SOURCE SETUP
# *****************************************************************************
# configure a header file to pass some of the CMake settings to the source code
configure_file (
    "${PROJECT_SOURCE_DIR}/mcp_config.h.in"
    "${PROJECT_BINARY_DIR}/include/mcp_config.h"
)

set(CONFIG_DIR "${PROJECT_SOURCE_DIR}/../blast_config")
set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")
set(COMMS_DIR "${PROJECT_SOURCE_DIR}/../blast_comms")
set(MPSSE_DIR "${PROJECT_SOURCE_DIR}/../external_libs/mpsse")
set(SYNCLINK_DIR "${PROJECT_SOURCE_DIR}/../external_libs/synclink")
set(COMMUNICATIONS_DIR "${PROJECT_SOURCE_DIR}/communications/")
set(LIBLINKLIST_DIR "${PROJECT_SOURCE_DIR}/../liblinklist")
set(UTILS_DIR "${PROJECT_SOURCE_DIR}/../utils")

set(ETC_DIR "${PROJECT_SOURCE_DIR}/../blast_etc")
FILE(GLOB etc_files "${ETC_DIR}/*.txt" "${ETC_DIR}/*.sch" "${ETC_DIR}/*.library" "${ETC_DIR}/*.lut" "${ETC_DIR}/WMM.COF")

add_subdirectory(${CONFIG_DIR} "${PROJECT_BINARY_DIR}/blast_config")
# add commanding library
add_subdirectory("${PROJECT_SOURCE_DIR}/commanding")
# add mcp communication library
add_subdirectory("${PROJECT_SOURCE_DIR}/communications")
# add pointing library
add_subdirectory("${PROJECT_SOURCE_DIR}/pointing")
# add motors library
add_subdirectory("${PROJECT_SOURCE_DIR}/motors")
# add sensors library
add_subdirectory("${PROJECT_SOURCE_DIR}/sensors")
# add housekeeping library
add_subdirectory("${PROJECT_SOURCE_DIR}/housekeeping")
# add diskmanager library
add_subdirectory("${PROJECT_SOURCE_DIR}/diskmanager")
# add data frame library
add_subdirectory("${PROJECT_SOURCE_DIR}/framing")
# add common library
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")
# add asynchronous communication library
add_subdirectory(${COMMS_DIR} "${PROJECT_BINARY_DIR}/blast_comms")
# add MPSSE Files
add_subdirectory(${MPSSE_DIR} "${PROJECT_BINARY_DIR}/mpsse")
# add common linklist library
set(LL_BUILD_LIBRARY ON)
add_subdirectory(${LIBLINKLIST_DIR} "${PROJECT_BINARY_DIR}/liblinklist")
# add utils library
add_subdirectory(${UTILS_DIR} "${PROJECT_BINARY_DIR}/utils")

# actually build mcp executable
add_executable(mcp
    mcp.c
    scheduler_tng.c
    tx.c
    ${PROJECT_SOURCE_DIR}/include/mcp.h
    ${PROJECT_SOURCE_DIR}/include/scheduler_tng.h
    ${PROJECT_SOURCE_DIR}/include/tx.h
)

add_dependencies(mcp soem phenomproj)

target_include_directories(mcp PRIVATE
    ${PH_INCLUDE_DIR}
)

target_link_libraries(mcp PRIVATE
    blast_common
    command_common
    command_list
    commanding
    diskmanager
    framing
    housekeeping
    linklist
    motors
    pointing
    mcp_sensors
    tx_struct
    utils
    ${PH_LIBRARY}
    ssl
    m
)

if (ENABLE_STYLE_CHECK)
    check_style(mcp)
endif()


# *****************************************************************************
# INSTALL
# *****************************************************************************
install(TARGETS mcp DESTINATION bin)
install(DIRECTORY DESTINATION "/data/etc")
install(FILES ${etc_files} DESTINATION "/data/etc/blast")
