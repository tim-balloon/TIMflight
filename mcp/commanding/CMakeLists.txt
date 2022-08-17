add_library(commanding STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/commands.c
)
define_file_basename_for_sources(commanding)

target_include_directories(commanding PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../include
)
target_link_libraries(commanding PRIVATE
    blast_common
    command_common
    housekeeping
    motors
    utils
    ${PH_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(commanding)
endif()