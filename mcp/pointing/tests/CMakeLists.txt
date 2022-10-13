find_package(cmocka CONFIG)

add_executable(test_pointing
    test_pointing.c
)

add_dependencies(test_pointing libcmocka)

target_include_directories(test_pointing PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)
target_link_libraries(test_pointing PRIVATE
    ${CMOCKA_LIBRARY}
    pointing
)

if (ENABLE_STYLE_CHECK)
    check_style(test_pointing)
endif()

add_test(NAME pointing COMMAND test_pointing 1)
