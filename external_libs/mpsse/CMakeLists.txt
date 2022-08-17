add_library (mpsse STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mpsse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/binarybuffer.c
)
define_file_basename_for_sources(mpsse)

target_include_directories(mpsse PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(mpsse PRIVATE
    blast_common
    ${USB_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(mpsse)
endif()