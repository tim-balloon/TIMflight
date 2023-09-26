find_package(cmocka CONFIG)

# EVTM code

add_executable(test_gh_evtm
    ${CMAKE_CURRENT_LIST_DIR}/test_gh_evtm.c
)

add_dependencies(test_gh_evtm libcmocka)

target_include_directories(test_gh_evtm PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)
target_link_libraries(test_gh_evtm PRIVATE
    ${CMOCKA_LIBRARY}
    blast
)

# last element SHOULD NOT have a comma, else you will get a linker error
# "/usr/bin/ld: cannot find : No such file or directory"
string(CONCAT LINK_FLAGS_WRAP
    "-Wl"
)

# wrapping initBITSender
# compiler will replace all calls to initBITSender with __wrap_initBITSender, and so on...
set_property(TARGET test_gh_evtm
    PROPERTY
        LINK_FLAGS
            ${DEFAULT_LINK_FLAGS} ${LINK_FLAGS_WRAP})

if (ENABLE_STYLE_CHECK)
    check_style(test_gh_evtm)
endif()

add_test(NAME groundhog_evtm COMMAND test_gh_evtm 1)

# Run unit test upon build
if (ENABLE_TESTS_ON_BUILD)
    set(UNIT_TEST test_gh_evtm)
    add_custom_command(
        TARGET ${UNIT_TEST}
        COMMENT "Run test_gh_evtm"
        POST_BUILD
        COMMAND ${UNIT_TEST}
    )
endif()