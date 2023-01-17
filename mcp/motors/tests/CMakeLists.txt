find_package(cmocka CONFIG)

add_executable(test_motors
    ${CMAKE_CURRENT_LIST_DIR}/test_balance.c
)

add_dependencies(test_motors libcmocka)

target_include_directories(test_motors PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing # get mock mcp defines
)
target_link_libraries(test_motors PRIVATE
    ${CMOCKA_LIBRARY}
    motors
)

# Enable mocking EZStepper funcs
set_property(TARGET test_motors
    PROPERTY
        LINK_FLAGS
            "${DEFAULT_LINK_FLAGS} -Wl,--wrap=EZBus_ReadInt")

if (ENABLE_STYLE_CHECK)
    check_style(test_motors)
endif()

add_test(NAME motors COMMAND test_motors 1)

# Run unit test upon build
set(UNIT_TEST test_motors)
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run test_motors"
     POST_BUILD
     COMMAND ${UNIT_TEST}
)