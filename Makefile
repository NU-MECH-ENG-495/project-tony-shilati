# Add the path to the Eigen library
EIGEN3_INCLUDE_DIR = /opt/homebrew/include/eigen3

# Add the include directory to the compiler flags
CXXFLAGS += -I$(EIGEN3_INCLUDE_DIR)

finger_model: src/finger_model.cpp
	$(CXX) $(CXXFLAGS) -o finger_model src/finger_model.cpp
