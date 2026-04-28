add_library(rfsoc STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/rfsoc_commanding.c
)
define_file_basename_for_sources(rfsoc)

target_include_directories(rfsoc PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../include
)
target_link_libraries(rfsoc PRIVATE
    blast_common
    command_common
    housekeeping
    motors
    utils
    ${PH_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(rfsoc)
endif()
