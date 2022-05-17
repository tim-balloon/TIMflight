find_package(cmocka)

add_executable(test_utils
        test_comparison.c
        "$<TARGET_OBJECTS:utils>"
)
target_include_directories(test_utils PRIVATE
        ${CMOCKA_INCLUDE_DIR}
        ../include
)
target_link_libraries(test_utils
        m
        ${CMOCKA_LIBRARY}
)

add_test(NAME utils COMMAND test_utils 1)