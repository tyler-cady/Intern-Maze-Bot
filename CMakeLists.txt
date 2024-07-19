cmake_minimum_required(VERSION 3.10)
project(MouseTests)

# Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Add your source files here
set(SOURCES mouse.cpp)

# Add the test executable
add_executable(MouseTests ${SOURCES} mouse_test.cpp)

# Link Google Test libraries
target_link_libraries(MouseTests ${GTEST_LIBRARIES} pthread)