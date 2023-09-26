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
        COMMAND ${PROJECT_SOURCE_DIR}/tests/clint.sh ${check_files}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        COMMENT "Checking Code Style in ${targetname}"
        VERBATIM
    )
endfunction()

project (groundhog)

cmake_minimum_required (VERSION 2.8.8)

# *****************************************************************************
# PROJECT DEFINITIONS
# *****************************************************************************
option(ENABLE_TESTING "build unit tests" ON)
option(ENABLE_TESTS_ON_BUILD "run tests during build" ON)
option(ENABLE_STYLE_CHECK "Run Google cpplint" ON)
option(ENABLE_COVERAGE "Run gcov" OFF)

SET (CMAKE_C_FLAGS                  "-Wall -std=gnu99")
SET (CMAKE_C_FLAGS_DEBUG            "-g")
SET (CMAKE_C_FLAGS_MINSIZEREL       "-Os -DNDEBUG")
SET (CMAKE_C_FLAGS_RELEASE          "-O3 -DNDEBUG")
SET (CMAKE_C_FLAGS_RELWITHDEBINFO   "-O2 -g")

SET (CMAKE_CXX_FLAGS                "-Wall")
SET (CMAKE_CXX_FLAGS_DEBUG          "-g")
SET (CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELEASE        "-O3 -DNDEBUG")
SET (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")

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
find_package (Threads)

# Find ExternalProject module
include (ExternalProject)

find_path(BLAST_CONFIG NAMES tx_struct_tng.c
    HINTS ${PROJECT_SOURCE_DIR}/../blast_config REQUIRED)
IF(BLAST_CONFIG)
    message("Groundhog found blast_config")
    set(MAKE_BLAST True)
    add_definitions(-DMAKE_BLAST)
    include(${PROJECT_SOURCE_DIR}/blast/CMakeLists.txt)
ENDIF()

if (ENABLE_TESTING)
    include(ExternalProject)
    ExternalProject_Add(libcmocka
        GIT_REPOSITORY https://gitlab.com/cmocka/cmocka
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


# Find getdata
pkg_check_modules(PC_GD QUIET getdata)
find_path(GD_INCLUDE_DIR NAMES getdata.h
    HINTS ${PC_GD_INCLUDEDIR} ${PC_GD_INCLUDE_DIRS} REQUIRED)
find_library(GD_LIBRARY NAMES getdata
    HINTS ${PC_GD_LIBDIR} ${PC_GD_LIBRARY_DIRS} REQUIRED)
IF(NOT GD_INCLUDE_DIR)
    MESSAGE(FATAL_ERROR "Did not find getdata.h.  Is libgetdata-dev installed?")
ENDIF()
IF(NOT GD_LIBRARY)
    MESSAGE(FATAL_ERROR "Did not find libgetdata.  Is libgetdata4 installed?")
ENDIF()
include_directories(${GD_INCLUDE_DIR})

set(LIBLINKLIST_DIR "${PROJECT_SOURCE_DIR}/../liblinklist")

# add machine directory for OSX builds (needs endian.h)
include_directories ("/usr/include/machine")
# add the binary tree to the search path for include files so that we will find decomd_config.h
include_directories (
    "${LIBLINKLIST_DIR}"
    "${PROJECT_BINARY_DIR}/include"
    "${PROJECT_SOURCE_DIR}"
)

# add common linklist objects
set(LL_BUILD_LIBRARY ON)
add_subdirectory(${LIBLINKLIST_DIR} "${PROJECT_BINARY_DIR}/liblinklist")
get_target_property(LL_SOURCES linklist SOURCES)
add_library (liblinklist OBJECT ${LL_SOURCES})
define_file_basename_for_sources(liblinklist)
set(COMMON_OBJECTS
    $<TARGET_OBJECTS:liblinklist>
    ${PROJECT_SOURCE_DIR}/groundhog_funcs.c
    ${PROJECT_SOURCE_DIR}/groundhog_funcs.h
)


IF(NOT MAKE_BLAST)
    message("No _config directories found. No executable compiled.")
ENDIF()

install(DIRECTORY DESTINATION "/data/rawdir"
    DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE)
install(DIRECTORY DESTINATION "/data/groundhog"
    DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE)
install(DIRECTORY DESTINATION "/data/etc"
    DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_WRITE WORLD_EXECUTE)

if (ENABLE_TESTING)
    add_subdirectory(tests)
endif(ENABLE_TESTING)