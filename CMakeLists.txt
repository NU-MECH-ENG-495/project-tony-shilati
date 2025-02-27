cmake_minimum_required(VERSION 3.28)
project(ME_495_Final_Project VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
add_library(${PROJECT_NAME}
    src/render_finger.cpp
)

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
)

# Fetch SFML 3.0
include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

# Add the executable
add_executable(main src/main.cpp)

# Link the necessary SFML libraries
target_compile_features(main PRIVATE cxx_std_17)
target_link_libraries(main PRIVATE
    SFML::Graphics
    SFML::Window
    SFML::System
)

# Enable testing
enable_testing()

# Add the tests subdirectory (if you have tests)
add_subdirectory(tests)
