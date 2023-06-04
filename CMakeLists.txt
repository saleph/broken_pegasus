cmake_minimum_required(VERSION 3.5)
project(broken_pegasus)

set(CMAKE_TOOLCHAIN_FILE "conan_toolchain.cmake")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

find_package(GTest REQUIRED)

enable_testing()
file(GLOB APP_SOURCES "src/**.cpp")
file(GLOB TEST_SOURCES "tests/**.cpp")

add_library(app STATIC 
    ${APP_SOURCES}
)
target_link_libraries(
    app
)
add_executable(
    ${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/main.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    app
)

add_executable(
    tests_${PROJECT_NAME}
    ${TEST_SOURCES}
)
target_link_libraries(
    tests_${PROJECT_NAME}
    app
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(tests_${PROJECT_NAME})
