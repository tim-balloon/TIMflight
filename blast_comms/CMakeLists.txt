add_library(blast_comms STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/comms_netasync.c
    ${CMAKE_CURRENT_SOURCE_DIR}/comms_netbuf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/comms_netsock.c
    ${CMAKE_CURRENT_SOURCE_DIR}/comms_serial.c
)
define_file_basename_for_sources(blast_comms)
target_include_directories(blast_comms PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(blast_comms PRIVATE
    blast_common
)

if (ENABLE_STYLE_CHECK)
    check_style(blast_comms)
endif()
