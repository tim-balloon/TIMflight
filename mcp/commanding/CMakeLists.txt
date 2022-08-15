add_library(commanding STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/commands.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/command_struct.h
)
define_file_basename_for_sources(commanding)

target_include_directories(commanding PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/.. # mcp.c circular dependency
)
target_link_libraries(commanding PUBLIC
    blast_common
    housekeeping
    motors
    roach
    utils
)

if (ENABLE_STYLE_CHECK)
    check_style(commanding)
endif()