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

# Find Qt5
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add the open_chain_kinematics library
add_library(open_chain_kinematics
    src/open_chain_kinematics.cpp
)

target_include_directories(open_chain_kinematics PUBLIC
    ${PROJECT_SOURCE_DIR}/src/include
)

target_link_libraries(open_chain_kinematics PUBLIC
    Eigen3::Eigen
)

# Add the finger_model library
add_library(finger_model
    src/finger_model.cpp
)

target_include_directories(finger_model PUBLIC
    ${PROJECT_SOURCE_DIR}/src/include
)

target_link_libraries(finger_model PUBLIC
    Eigen3::Eigen
)

# Add the executables
add_executable(main src/main.cpp src/render_finger.cpp src/finger_model.cpp)

# Link the necessary Qt and Eigen libraries
target_compile_features(main PRIVATE cxx_std_17)
target_link_libraries(main PRIVATE
    Qt5::Widgets
    Eigen3::Eigen
    open_chain_kinematics
    finger_model
)

# Link Qt to the project library
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt5::Widgets
)

# Enable testing
enable_testing()

# Add the tests subdirectory
add_subdirectory(tests)

# Find Doxygen
find_package(Doxygen)
if (DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(doc_doxygen
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif()
