find_package(cmocka CONFIG)

add_executable(test_utils
    test_comparison.c
)

add_dependencies(test_utils libcmocka)

target_include_directories(test_utils PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ../include
)
target_link_libraries(test_utils PRIVATE
    ${CMOCKA_LIBRARY}
    utils
    m
)

add_test(NAME utils COMMAND test_utils 1)