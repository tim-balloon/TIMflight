add_library (mcp_sensors STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/acs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/csbf_dgps.c
    ${CMAKE_CURRENT_SOURCE_DIR}/dsp1760.c
    ${CMAKE_CURRENT_SOURCE_DIR}/gps.c
    ${CMAKE_CURRENT_SOURCE_DIR}/geomag.c
    ${CMAKE_CURRENT_SOURCE_DIR}/inclinometer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/magnetometer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/socket_utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/star_camera_receive.c
    ${CMAKE_CURRENT_SOURCE_DIR}/star_camera_solutions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/star_camera_transmit.c
    ${CMAKE_CURRENT_SOURCE_DIR}/star_camera_trigger.c
    ${CMAKE_CURRENT_SOURCE_DIR}/xsc_network.c
    ${CMAKE_CURRENT_SOURCE_DIR}/xsc_protocol.c
)
define_file_basename_for_sources(mcp_sensors)

target_include_directories(mcp_sensors PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(mcp_sensors PRIVATE
    blast_comms
    ${PH_LIBRARY}
    ssl
)

if (ENABLE_STYLE_CHECK)
    set(exclude_patterns "geomag[0-9]*\\.[ch]" "EGM[0-9]+\\.h")
    check_style(mcp_sensors ${exclude_patterns})
endif()
