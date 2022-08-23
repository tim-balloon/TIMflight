add_library(utils STATIC
    comparison.c
)
target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if (ENABLE_STYLE_CHECK)
    check_style(utils)
endif()

if (ENABLE_TESTING)
    add_subdirectory(tests)
endif(ENABLE_TESTING)
