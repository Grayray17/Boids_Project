# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/cmake-build-debug"

# Utility rule file for res.

# Include any custom commands dependencies for this target.
include res/CMakeFiles/res.dir/compiler_depend.make

# Include the progress variables for this target.
include res/CMakeFiles/res.dir/progress.make

res: res/CMakeFiles/res.dir/build.make
.PHONY : res

# Rule to build all files generated by this target.
res/CMakeFiles/res.dir/build: res
.PHONY : res/CMakeFiles/res.dir/build

res/CMakeFiles/res.dir/clean:
	cd "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/cmake-build-debug/res" && $(CMAKE_COMMAND) -P CMakeFiles/res.dir/cmake_clean.cmake
.PHONY : res/CMakeFiles/res.dir/clean

res/CMakeFiles/res.dir/depend:
	cd "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/cmake-build-debug" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work" "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/res" "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/cmake-build-debug" "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/cmake-build-debug/res" "/Users/matias/Desktop/Uni/Second Year/Cgra251/Assignment 3/CGRA251_Framework_A3(1)/work/cmake-build-debug/res/CMakeFiles/res.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : res/CMakeFiles/res.dir/depend

