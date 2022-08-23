add_library(blast_common STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/bitserver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/blast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/channels_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CRC_func.c
    ${CMAKE_CURRENT_SOURCE_DIR}/fir.c
    ${CMAKE_CURRENT_SOURCE_DIR}/lut.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mputs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/pq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/PMurHash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sip.c
    ${CMAKE_CURRENT_SOURCE_DIR}/slowdl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/xsc_fifo.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FIFO.c
    ${CMAKE_CURRENT_SOURCE_DIR}/logger.c
)
define_file_basename_for_sources(blast_common)

target_include_directories(blast_common PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${GLIB_INCLUDE_DIR}
    ${GLIBCONFIG_INCLUDE_DIR}
    ${PH_INCLUDE_DIR}/..
)
target_link_libraries(blast_common PRIVATE
    command_list
    commanding
    linklist
)

if (ENABLE_STYLE_CHECK)
    check_style(blast_common)
endif()

set(GROUNDHOG_COMMON_SRCS
    ${GROUNDHOG_COMMON_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/blast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/bitserver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/channels_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CRC_func.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FIFO.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mputs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/PMurHash.c
    PARENT_SCOPE
)
set(GROUNDHOG_COMMON_HEADERS
   ${GROUNDHOG_COMMON_HEADERS}
   ${CMAKE_CURRENT_SOURCE_DIR}/include/blast.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/blast_time.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/bitserver.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/channels_tng.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/crc.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/CRC_func.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/derived.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/FIFO.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/mputs.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/PMurHash.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/portable_endian.h
   PARENT_SCOPE
)

set(DECOM_COMMON_SRCS
    ${DECOM_COMMON_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/blast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/channels_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mputs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/PMurHash.c
    PARENT_SCOPE
)
set(DECOM_COMMON_HEADERS
   ${DECOM_COMMON_HEADERS}
   ${CMAKE_CURRENT_SOURCE_DIR}/include/blast.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/blast_time.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/channels_tng.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/crc.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/derived.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/mputs.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/PMurHash.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/portable_endian.h
   PARENT_SCOPE
)

set(DEFRICHER_COMMON_SRCS
    ${DEFRICHER_COMMON_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/blast.c
    ${CMAKE_CURRENT_SOURCE_DIR}/channels_tng.c
    ${CMAKE_CURRENT_SOURCE_DIR}/crc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/CRC_func.c
    ${CMAKE_CURRENT_SOURCE_DIR}/FIFO.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mputs.c
    ${CMAKE_CURRENT_SOURCE_DIR}/PMurHash.c
    PARENT_SCOPE
)
set(DEFRICHER_COMMON_HEADERS
   ${DEFRICHER_COMMON_HEADERS}
   ${CMAKE_CURRENT_SOURCE_DIR}/include/blast.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/blast_time.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/channels_tng.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/crc.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/CRC_func.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/derived.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/FIFO.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/mputs.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/PMurHash.h
   ${CMAKE_CURRENT_SOURCE_DIR}/include/portable_endian.h
   PARENT_SCOPE
)

add_library(command_common STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/command_common.c
)
define_file_basename_for_sources(command_common)

target_include_directories(command_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(command_common PRIVATE
    command_list
)

if (ENABLE_STYLE_CHECK)
    check_style(command_common)
endif()
