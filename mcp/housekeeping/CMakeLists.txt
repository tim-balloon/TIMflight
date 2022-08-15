add_library(housekeeping STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/auxiliary.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bias_tone.c
    ${CMAKE_CURRENT_SOURCE_DIR}/chrgctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/computer_sensors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cryostat.c
    ${CMAKE_CURRENT_SOURCE_DIR}/data_sharing_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/hawkeyeir.c
    ${CMAKE_CURRENT_SOURCE_DIR}/highbay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/labjack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/labjack_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/microscroll.c
    ${CMAKE_CURRENT_SOURCE_DIR}/multiplexed_labjack.c
    ${CMAKE_CURRENT_SOURCE_DIR}/outer_frame.c
    ${CMAKE_CURRENT_SOURCE_DIR}/relay_control.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_updates.c
    ${CMAKE_CURRENT_SOURCE_DIR}/therm_heater.c
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog.c
    ${CMAKE_CURRENT_SOURCE_DIR}/therm_roach.c
)
define_file_basename_for_sources(housekeeping)
target_include_directories(housekeeping PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(housekeeping PUBLIC
    communications
    ${ALSA_LIBRARY}
    ${MODBUS_LIBRARY}
    ${SENSORS_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(housekeeping)
endif()