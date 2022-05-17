add_library(utils OBJECT
    comparison.c
)
target_include_directories(utils PUBLIC .)

if (ENABLE_TESTING)
    add_subdirectory(tests)
endif(ENABLE_TESTING)
