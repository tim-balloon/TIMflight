add_library(housekeeping STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/auxiliary.c
    ${CMAKE_CURRENT_SOURCE_DIR}/chrgctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/computer_sensors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/data_sharing_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/labjack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/labjack_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/labjack_test.c
    ${CMAKE_CURRENT_SOURCE_DIR}/multiplexed_labjack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/therm_heater.c
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog.c
)
define_file_basename_for_sources(housekeeping)

add_dependencies(housekeeping phenomproj)

target_include_directories(housekeeping PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${PH_INCLUDE_DIR}/..
)
target_link_libraries(housekeeping PRIVATE
    blast_common
    communications
    ${MODBUS_LIBRARY}
    ${PH_LIBRARY}
    ${SENSORS_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(housekeeping)
endif()
