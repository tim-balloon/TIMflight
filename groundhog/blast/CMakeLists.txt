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
include_directories(${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})


set(COMMON_DIR "${PROJECT_SOURCE_DIR}/../common")
set(COMMUNICATIONS_DIR "${PROJECT_SOURCE_DIR}/../mcp/communications")
set(CONFIG_DIR "${PROJECT_SOURCE_DIR}/../blast_config")
set(COMMS_DIR "${PROJECT_SOURCE_DIR}/../blast_comms")
set(MCP_DIR "${PROJECT_SOURCE_DIR}/../mcp")

include_directories (
    "${COMMON_DIR}/include"
    "${COMMS_DIR}/include"
    "${CONFIG_DIR}/include"
    "${COMMUNICATIONS_DIR}/include"
    "${MCP_DIR}/../mcp/include"
)

# add common objects 
add_subdirectory(${COMMON_DIR} "${PROJECT_BINARY_DIR}/blast_common")
add_subdirectory(${CONFIG_DIR} "${PROJECT_BINARY_DIR}/blast_config")
# add asynchronous communication objects
add_subdirectory(${COMMS_DIR} "${PROJECT_BINARY_DIR}/blast_comms")

add_library (blast_common_groundhog OBJECT ${GROUNDHOG_COMMON_SRCS} ${GROUNDHOG_COMMON_HEADERS})
define_file_basename_for_sources(blast_common_groundhog)

get_target_property(COMMS_SRCS blast_comms SOURCES)
add_library (blast_comms_groundhog OBJECT ${COMMS_SRCS})
define_file_basename_for_sources(blast_comms_groundhog)

add_executable (groundhog 
    $<TARGET_OBJECTS:liblinklist>
    ${PROJECT_SOURCE_DIR}/groundhog_funcs.c
    ${PROJECT_SOURCE_DIR}/groundhog_funcs.h

    $<TARGET_OBJECTS:blast_common_groundhog>
    $<TARGET_OBJECTS:blast_comms_groundhog>
    ${PROJECT_SOURCE_DIR}/blast/main.c
    ${PROJECT_SOURCE_DIR}/blast/decom_pci.h
    ${PROJECT_SOURCE_DIR}/blast/groundhog.h
    ${PROJECT_SOURCE_DIR}/blast/biphase.c
    ${PROJECT_SOURCE_DIR}/blast/highrate.c
    ${PROJECT_SOURCE_DIR}/blast/evtm.c
    ${PROJECT_SOURCE_DIR}/blast/pilot.c
)

target_link_libraries(groundhog
    ${GLIB_LIBRARY}
    ${GD_LIBRARY}
    ${CMAKE_THREAD_LIBS_INIT}
    ssl crypto
)

if (ENABLE_STYLE_CHECK)
    check_style(groundhog)
endif()

install(TARGETS groundhog DESTINATION bin)

if (ENABLE_TESTING)
    add_subdirectory(${PROJECT_SOURCE_DIR}/blast/tests/)
endif(ENABLE_TESTING)
