find_package(cmocka CONFIG)

# =============================================================================
# Balance system
# =============================================================================
add_executable(test_balance
    ${CMAKE_CURRENT_LIST_DIR}/test_balance.c
)

add_dependencies(test_balance libcmocka)

target_include_directories(test_balance PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing # get mock mcp defines
)
target_link_libraries(test_balance PRIVATE
    ${CMOCKA_LIBRARY}
    motors
)

# Enable mocking EZStepper funcs
set_property(TARGET test_balance
    PROPERTY
        LINK_FLAGS
            "${DEFAULT_LINK_FLAGS} -Wl,--wrap=EZBus_ReadInt")

if (ENABLE_STYLE_CHECK)
    check_style(test_balance)
endif()

add_test(NAME test_balance COMMAND test_balance 1)

# Run unit test upon build
set(UNIT_TEST test_balance)
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run test_balance"
     POST_BUILD
     COMMAND ${UNIT_TEST}
)

# =============================================================================
# Shutter
# =============================================================================
add_executable(test_shutter
    ${CMAKE_CURRENT_LIST_DIR}/test_shutter.c
)

add_dependencies(test_shutter libcmocka)

target_include_directories(test_shutter PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing
)
target_link_libraries(test_shutter PRIVATE
    ${CMOCKA_LIBRARY}
    motors
)

# Enable mocking EZStepper funcs
string(CONCAT LINK_FLAGS_WRAP
    "-Wl,"
    "--wrap=EZBus_SetIHold,"
    "--wrap=EZBus_SetIMove,"
    "--wrap=EZBus_SetVel,"
    "--wrap=EZBus_SetAccel,"
    "--wrap=EZBus_MoveComm,"
    "--wrap=EZBus_Take,"
    "--wrap=EZBus_Stop,"
    "--wrap=EZBus_Release"
)
set_property(TARGET test_shutter
    PROPERTY
        LINK_FLAGS
             ${DEFAULT_LINK_FLAGS} ${LINK_FLAGS_WRAP})

if (ENABLE_STYLE_CHECK)
    check_style(test_shutter)
endif()

add_test(NAME test_shutter COMMAND test_shutter 1)

# Run unit test upon build
set(UNIT_TEST test_shutter)
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run test_shutter"
     POST_BUILD
     COMMAND ${UNIT_TEST}
)

# =============================================================================
# elevation axis lock
# =============================================================================
add_executable(test_el_lock
    ${CMAKE_CURRENT_LIST_DIR}/test_el_lock.c
)

add_dependencies(test_el_lock libcmocka)

target_include_directories(test_el_lock PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing
)
target_link_libraries(test_el_lock PRIVATE
    ${CMOCKA_LIBRARY}
    motors
)

# Enable mocking EZStepper funcs
set_property(TARGET test_el_lock
    PROPERTY
        LINK_FLAGS
            "${DEFAULT_LINK_FLAGS} -Wl,--wrap=EZBus_IsTaken,--wrap=EZBus_Comm,--wrap=EZBus_Stop,--wrap=EZBus_RelMove")

if (ENABLE_STYLE_CHECK)
    check_style(test_el_lock)
endif()

add_test(NAME test_el_lock COMMAND test_el_lock 1)

# Run unit test upon build
set(UNIT_TEST test_el_lock)
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run test_el_lock"
     POST_BUILD
     COMMAND ${UNIT_TEST}
)