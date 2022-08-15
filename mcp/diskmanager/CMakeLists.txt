add_library (diskmanager STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/store_data.c
    ${CMAKE_CURRENT_SOURCE_DIR}/diskmanager_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/file_buffer_tng.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../include/store_data.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/diskmanager_tng.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/file_buffer_tng.h
)
define_file_basename_for_sources(diskmanager)

target_link_libraries(diskmanager PUBLIC
    blast_common
    pthread
    ${CK_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(diskmanager)
endif()