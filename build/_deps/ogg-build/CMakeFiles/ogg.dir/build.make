# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build

# Include any dependencies generated for this target.
include _deps/ogg-build/CMakeFiles/ogg.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include _deps/ogg-build/CMakeFiles/ogg.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/ogg-build/CMakeFiles/ogg.dir/progress.make

# Include the compile flags for this target's objects.
include _deps/ogg-build/CMakeFiles/ogg.dir/flags.make

_deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.o: _deps/ogg-build/CMakeFiles/ogg.dir/flags.make
_deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.o: _deps/ogg-src/src/bitwise.c
_deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.o: _deps/ogg-build/CMakeFiles/ogg.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object _deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.o"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT _deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.o -MF CMakeFiles/ogg.dir/src/bitwise.c.o.d -o CMakeFiles/ogg.dir/src/bitwise.c.o -c /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src/src/bitwise.c

_deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/ogg.dir/src/bitwise.c.i"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src/src/bitwise.c > CMakeFiles/ogg.dir/src/bitwise.c.i

_deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/ogg.dir/src/bitwise.c.s"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src/src/bitwise.c -o CMakeFiles/ogg.dir/src/bitwise.c.s

_deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.o: _deps/ogg-build/CMakeFiles/ogg.dir/flags.make
_deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.o: _deps/ogg-src/src/framing.c
_deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.o: _deps/ogg-build/CMakeFiles/ogg.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object _deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.o"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT _deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.o -MF CMakeFiles/ogg.dir/src/framing.c.o.d -o CMakeFiles/ogg.dir/src/framing.c.o -c /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src/src/framing.c

_deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/ogg.dir/src/framing.c.i"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src/src/framing.c > CMakeFiles/ogg.dir/src/framing.c.i

_deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/ogg.dir/src/framing.c.s"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src/src/framing.c -o CMakeFiles/ogg.dir/src/framing.c.s

# Object files for target ogg
ogg_OBJECTS = \
"CMakeFiles/ogg.dir/src/bitwise.c.o" \
"CMakeFiles/ogg.dir/src/framing.c.o"

# External object files for target ogg
ogg_EXTERNAL_OBJECTS =

_deps/sfml-build/lib/libogg.a: _deps/ogg-build/CMakeFiles/ogg.dir/src/bitwise.c.o
_deps/sfml-build/lib/libogg.a: _deps/ogg-build/CMakeFiles/ogg.dir/src/framing.c.o
_deps/sfml-build/lib/libogg.a: _deps/ogg-build/CMakeFiles/ogg.dir/build.make
_deps/sfml-build/lib/libogg.a: _deps/ogg-build/CMakeFiles/ogg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library ../sfml-build/lib/libogg.a"
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && $(CMAKE_COMMAND) -P CMakeFiles/ogg.dir/cmake_clean_target.cmake
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ogg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
_deps/ogg-build/CMakeFiles/ogg.dir/build: _deps/sfml-build/lib/libogg.a
.PHONY : _deps/ogg-build/CMakeFiles/ogg.dir/build

_deps/ogg-build/CMakeFiles/ogg.dir/clean:
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build && $(CMAKE_COMMAND) -P CMakeFiles/ogg.dir/cmake_clean.cmake
.PHONY : _deps/ogg-build/CMakeFiles/ogg.dir/clean

_deps/ogg-build/CMakeFiles/ogg.dir/depend:
	cd /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-src /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build /Users/tonyshilati/ME495_C++/Final_Project/project-tony-shilati/build/_deps/ogg-build/CMakeFiles/ogg.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/ogg-build/CMakeFiles/ogg.dir/depend

