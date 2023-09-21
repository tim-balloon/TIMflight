add_library (communications STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/bi0.c
    ${CMAKE_CURRENT_SOURCE_DIR}/biphase_hardware.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pilot.c
    ${CMAKE_CURRENT_SOURCE_DIR}/highrate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/evtm.c
)
define_file_basename_for_sources(communications)

add_dependencies(communications phenomproj)

target_include_directories(communications PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${SYNCLINK_DIR}
    ${PH_INCLUDE_DIR}/..
)
target_link_libraries(communications PRIVATE
    blast_common
    blast_comms
    linklist
    mpsse
    ${PH_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(communications)
endif()
