# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yddraiggochfawr/Desktop/cis563/cispbam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release

# Include any dependencies generated for this target.
include Projects/eigen_example/CMakeFiles/eigen_example.dir/depend.make

# Include the progress variables for this target.
include Projects/eigen_example/CMakeFiles/eigen_example.dir/progress.make

# Include the compile flags for this target's objects.
include Projects/eigen_example/CMakeFiles/eigen_example.dir/flags.make

Projects/eigen_example/CMakeFiles/eigen_example.dir/main.cpp.o: Projects/eigen_example/CMakeFiles/eigen_example.dir/flags.make
Projects/eigen_example/CMakeFiles/eigen_example.dir/main.cpp.o: ../../Projects/eigen_example/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Projects/eigen_example/CMakeFiles/eigen_example.dir/main.cpp.o"
	cd /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/eigen_example.dir/main.cpp.o -c /home/yddraiggochfawr/Desktop/cis563/cispbam/Projects/eigen_example/main.cpp

Projects/eigen_example/CMakeFiles/eigen_example.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eigen_example.dir/main.cpp.i"
	cd /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yddraiggochfawr/Desktop/cis563/cispbam/Projects/eigen_example/main.cpp > CMakeFiles/eigen_example.dir/main.cpp.i

Projects/eigen_example/CMakeFiles/eigen_example.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eigen_example.dir/main.cpp.s"
	cd /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yddraiggochfawr/Desktop/cis563/cispbam/Projects/eigen_example/main.cpp -o CMakeFiles/eigen_example.dir/main.cpp.s

# Object files for target eigen_example
eigen_example_OBJECTS = \
"CMakeFiles/eigen_example.dir/main.cpp.o"

# External object files for target eigen_example
eigen_example_EXTERNAL_OBJECTS =

../../Projects/eigen_example/eigen_example: Projects/eigen_example/CMakeFiles/eigen_example.dir/main.cpp.o
../../Projects/eigen_example/eigen_example: Projects/eigen_example/CMakeFiles/eigen_example.dir/build.make
../../Projects/eigen_example/eigen_example: Projects/eigen_example/CMakeFiles/eigen_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../Projects/eigen_example/eigen_example"
	cd /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eigen_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Projects/eigen_example/CMakeFiles/eigen_example.dir/build: ../../Projects/eigen_example/eigen_example

.PHONY : Projects/eigen_example/CMakeFiles/eigen_example.dir/build

Projects/eigen_example/CMakeFiles/eigen_example.dir/clean:
	cd /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example && $(CMAKE_COMMAND) -P CMakeFiles/eigen_example.dir/cmake_clean.cmake
.PHONY : Projects/eigen_example/CMakeFiles/eigen_example.dir/clean

Projects/eigen_example/CMakeFiles/eigen_example.dir/depend:
	cd /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yddraiggochfawr/Desktop/cis563/cispbam /home/yddraiggochfawr/Desktop/cis563/cispbam/Projects/eigen_example /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example /home/yddraiggochfawr/Desktop/cis563/cispbam/Build/Release/Projects/eigen_example/CMakeFiles/eigen_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Projects/eigen_example/CMakeFiles/eigen_example.dir/depend

