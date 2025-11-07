cmake_minimum_required(VERSION 3.28)
project(rwa3 VERSION 1.0 LANGUAGES C CXX)


add_executable(rwa3_cpp src/main.cpp src/robot_control.cpp src/robot_types.cpp)
target_include_directories(rwa3_cpp PRIVATE include)

# Set C++17 standard for the targets
set_property(TARGET rwa3_cpp PROPERTY CXX_STANDARD 17)
set_property(TARGET rwa3_cpp PROPERTY CXX_STANDARD_REQUIRED ON)

# Common settings for all lectures
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-Wall -Wextra -pedantic-errors)


find_program(VALGRIND_EXECUTABLE valgrind)

# Only create the targets if valgrind is found
if(VALGRIND_EXECUTABLE)

    # --- Target for Memory Leak Checking (memcheck) ---
    add_custom_target(memcheck)

    add_custom_command(
        TARGET memcheck
        COMMAND ${VALGRIND_EXECUTABLE} --leak-check=full --track-origins=yes $<TARGET_FILE:rwa3_cpp>
        COMMENT "Running Valgrind Memcheck on main"
    )

    # Ensure executables are built before any tool runs
    add_dependencies(memcheck rwa3_cpp)

endif()
