add_library (motors STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/ec_motors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/motors.c
    ${CMAKE_CURRENT_SOURCE_DIR}/actuators.c
    ${CMAKE_CURRENT_SOURCE_DIR}/ezstep.c
    ${CMAKE_CURRENT_SOURCE_DIR}/hwpr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/xystage.c
    ${CMAKE_CURRENT_SOURCE_DIR}/balance.c
    ${CMAKE_CURRENT_SOURCE_DIR}/cryovalves.c

    ${CMAKE_CURRENT_SOURCE_DIR}/../include/ec_motors.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/motors.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/hwpr.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/ezstep.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/xystage.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/balance.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/cryovalves.h 
)
define_file_basename_for_sources(motors)

target_include_directories(motors PUBLIC
    ${PROJECT_SOURCE_DIR}/../external_libs/soem/soem
    ${PROJECT_SOURCE_DIR}/../external_libs/soem/osal
    ${PROJECT_SOURCE_DIR}/../external_libs/soem/oshw
)

target_link_libraries(motors PUBLIC
    housekeeping
    pointing
    ${ECAT_LIBRARY}
    ${GLIB_LIBRARY}
)

if (ENABLE_STYLE_CHECK)
    check_style(motors)
endif()