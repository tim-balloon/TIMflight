find_package(cmocka)

add_executable(test_pointing
    test_pointing.c
    ${PROJECT_SOURCE_DIR}/mcp.c
    ${PROJECT_SOURCE_DIR}/scheduler_tng.c
    ${PROJECT_SOURCE_DIR}/tx.c
)
message(${PROJECT_SOURCE_DIR}/../common/include)
target_include_directories(test_pointing PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    # mcp includes
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(test_pointing PRIVATE
    blast_common
    pointing
    ${CMOCKA_LIBRARY}
)

add_test(NAME pointing COMMAND test_pointing 1)