cmake_minimum_required(VERSION 3.23)

project(test_utils C)

include(CTest)

add_executable(${PROJECT_NAME}
               test_comparison.c ../comparison.c
              )
target_link_libraries(${PROJECT_NAME} m cmocka)
target_include_directories(${PROJECT_NAME} PRIVATE
                           ${CMOCKA_PUBLIC_INCLUDE_DIRS}
                           ../include
                           )
add_test(NAME utils COMMAND ${PROJECT_NAME})