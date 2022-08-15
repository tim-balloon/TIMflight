add_library (communications STATIC ${CMAKE_CURRENT_SOURCE_DIR}/bi0.c
    ${CMAKE_CURRENT_SOURCE_DIR}/biphase_hardware.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/highrate.c
)
define_file_basename_for_sources(communications)

target_include_directories(communications PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${SYNCLINK_DIR}
)
target_link_libraries(communications PUBLIC
    blast_comms
    mpsse
)

if (ENABLE_STYLE_CHECK)
    check_style(communications)
endif()