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

# Run unit test upon build
set(UNIT_TEST test_utils)
add_custom_command(
     TARGET ${UNIT_TEST}
     COMMENT "Run test_utils"
     POST_BUILD
     COMMAND ${UNIT_TEST}
)
