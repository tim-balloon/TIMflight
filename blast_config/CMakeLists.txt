add_library(command_list STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/command_list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/include/command_list.h
)
define_file_basename_for_sources(command_list)

target_include_directories(command_list PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(command_list PUBLIC
    diskmanager
)

if (ENABLE_STYLE_CHECK)
    check_style(command_list)
endif()

add_library(tx_struct STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/tx_struct_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/derived.c
    ${CMAKE_CURRENT_SOURCE_DIR}/slowdl_struct.c
)
define_file_basename_for_sources(tx_struct)

target_link_libraries(tx_struct PUBLIC
    blast_common
)

if (ENABLE_STYLE_CHECK)
    check_style(tx_struct)
endif()

set(GROUNDHOG_COMMON_SRCS
    ${GROUNDHOG_COMMON_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/tx_struct_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/derived.c
    PARENT_SCOPE
)
set(DECOM_COMMON_SRCS
    ${DECOM_COMMON_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/tx_struct_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/derived.c
    PARENT_SCOPE
)
