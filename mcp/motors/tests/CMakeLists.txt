find_package(cmocka CONFIG)

add_executable(test_motors
    test_balance.c
)

add_dependencies(test_motors libcmocka)

target_include_directories(test_motors PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)
target_link_libraries(test_motors PRIVATE
    ${CMOCKA_LIBRARY}
    motors
)

if (ENABLE_STYLE_CHECK)
    check_style(test_motors)
endif()

add_test(NAME motors COMMAND test_motors 1)
