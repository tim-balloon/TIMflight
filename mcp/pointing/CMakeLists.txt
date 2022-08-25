add_library (pointing STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/angles.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pointing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/radbox.c
    ${CMAKE_CURRENT_SOURCE_DIR}/time_lst.c
    ${CMAKE_CURRENT_SOURCE_DIR}/time_julian.c
    ${CMAKE_CURRENT_SOURCE_DIR}/time_nutation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/utilities_pointing.c
    ${CMAKE_CURRENT_SOURCE_DIR}/xsc_pointing.c
)
define_file_basename_for_sources(pointing)

target_include_directories(pointing PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(pointing PRIVATE
    blast_common
    framing
    mcp_sensors
    ${GSL_LIBRARY}
    ${GSL_CBLAS_LIBRARY}
    m
)

if (ENABLE_STYLE_CHECK)
    check_style(pointing)
endif()

if (ENABLE_TESTING)
    add_subdirectory(tests)
endif(ENABLE_TESTING)
