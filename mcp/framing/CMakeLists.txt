add_library (framing STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/framing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/framing.h
)
define_file_basename_for_sources(framing)

target_link_libraries(framing PUBLIC
    blast_common
    ${MOSQ_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(framing)
endif()