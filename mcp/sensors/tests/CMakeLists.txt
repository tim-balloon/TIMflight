find_package(cmocka CONFIG)

add_executable(test_inclinometer
    ${CMAKE_CURRENT_LIST_DIR}/test_inclinometer.c
)

add_dependencies(test_inclinometer libcmocka)

target_include_directories(test_inclinometer PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing # get mock mcp defines
)
target_link_libraries(test_inclinometer PRIVATE
    ${CMOCKA_LIBRARY}
    mcp_sensors
)

if (ENABLE_STYLE_CHECK)
    check_style(test_inclinometer)
endif()

add_test(NAME inclinometer COMMAND test_inclinometer 1)

# Run unit test upon build
if (ENABLE_TESTS_ON_BUILD)
    set(UNIT_TEST test_inclinometer)
    add_custom_command(
        TARGET ${UNIT_TEST}
        COMMENT "Run test_inclinometer"
        POST_BUILD
        COMMAND ${UNIT_TEST}
    )
endif()