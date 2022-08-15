add_library(roach STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/qdr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/remote_serial.c
    ${CMAKE_CURRENT_SOURCE_DIR}/roach.c
    ${CMAKE_CURRENT_SOURCE_DIR}/roach_udp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/roach_multiplex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/valon.c
)
define_file_basename_for_sources(roach)

target_include_directories(roach PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${KATCP_INCLUDE_DIR}
)
target_link_libraries(roach PUBLIC
    diskmanager
    ${FFTW3_LIBRARY}
    ${KATCP_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(roach)
endif()