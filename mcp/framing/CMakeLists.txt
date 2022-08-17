add_library (framing STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/framing.c
)
define_file_basename_for_sources(framing)

target_include_directories(framing PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/../include
)
target_link_libraries(framing PRIVATE
    blast_common
    ${MOSQ_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(framing)
endif()