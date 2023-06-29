find_package(cmocka CONFIG)

add_executable(test_pointing
    ${CMAKE_CURRENT_LIST_DIR}/test_pointing.c
)

add_dependencies(test_pointing libcmocka)

target_include_directories(test_pointing PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing # get mock mcp defines
)
target_link_libraries(test_pointing PRIVATE
    ${CMOCKA_LIBRARY}
    pointing
)

if (ENABLE_STYLE_CHECK)
    check_style(test_pointing)
endif()

add_test(NAME pointing COMMAND test_pointing 1)

# Run unit test upon build
if (ENABLE_TESTS_ON_BUILD)
    set(UNIT_TEST test_pointing)
    add_custom_command(
        TARGET ${UNIT_TEST}
        COMMENT "Run test_pointing"
        POST_BUILD
        COMMAND ${UNIT_TEST}
    )
endif()