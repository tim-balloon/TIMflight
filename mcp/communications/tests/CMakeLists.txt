find_package(cmocka CONFIG)

# EVTM code

add_executable(test_evtm
    ${CMAKE_CURRENT_LIST_DIR}/test_evtm.c
)

add_dependencies(test_evtm libcmocka)

target_include_directories(test_evtm PRIVATE
    ${CMOCKA_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${PROJECT_SOURCE_DIR}/testing # get mock mcp defines
)
target_link_libraries(test_evtm PRIVATE
    ${CMOCKA_LIBRARY}
    communications
)

# last element SHOULD NOT have a comma, else you will get a linker error
# "/usr/bin/ld: cannot find : No such file or directory"
string(CONCAT LINK_FLAGS_WRAP
    "-Wl,"
    "--wrap=initBITSender,"
    "--wrap=setBITSenderSerial,"
    "--wrap=setBITSenderFramenum,"
    "--wrap=sendToBITSender,"
    "--wrap=testing_evtm,"
    "--wrap=bprintf"
)

# wrapping initBITSender
# compiler will replace all calls to initBITSender with __wrap_initBITSender, and so on...
set_property(TARGET test_evtm
    PROPERTY
        LINK_FLAGS
            ${DEFAULT_LINK_FLAGS} ${LINK_FLAGS_WRAP})

if (ENABLE_STYLE_CHECK)
    check_style(test_evtm)
endif()

add_test(NAME evtm COMMAND test_evtm 1)

# Run unit test upon build
if (ENABLE_TESTS_ON_BUILD)
    set(UNIT_TEST test_evtm)
    add_custom_command(
        TARGET ${UNIT_TEST}
        COMMENT "Run test_evtm"
        POST_BUILD
        COMMAND ${UNIT_TEST}
    )
endif()