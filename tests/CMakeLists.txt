cmake_minimum_required(VERSION 3.10)

# Add GoogleTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/release-1.11.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Add test executable
add_executable(
  my_tests
  # Replace these with actual test files
  ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_kinematics.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_rigid_body_motion.cpp
  # ...add other test files here...
)

target_include_directories(
  my_tests PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/include
)

target_link_libraries(
  my_tests
  gtest_main
  ME_495_Final_Project
  open_chain_kinematics
)

include(GoogleTest)
gtest_discover_tests(my_tests)
