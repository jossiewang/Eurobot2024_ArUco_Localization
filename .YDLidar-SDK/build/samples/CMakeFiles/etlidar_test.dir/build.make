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
CMAKE_SOURCE_DIR = /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build

# Include any dependencies generated for this target.
include samples/CMakeFiles/etlidar_test.dir/depend.make

# Include the progress variables for this target.
include samples/CMakeFiles/etlidar_test.dir/progress.make

# Include the compile flags for this target's objects.
include samples/CMakeFiles/etlidar_test.dir/flags.make

samples/CMakeFiles/etlidar_test.dir/etlidar_test.cpp.o: samples/CMakeFiles/etlidar_test.dir/flags.make
samples/CMakeFiles/etlidar_test.dir/etlidar_test.cpp.o: ../samples/etlidar_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object samples/CMakeFiles/etlidar_test.dir/etlidar_test.cpp.o"
	cd /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/etlidar_test.dir/etlidar_test.cpp.o -c /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/samples/etlidar_test.cpp

samples/CMakeFiles/etlidar_test.dir/etlidar_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/etlidar_test.dir/etlidar_test.cpp.i"
	cd /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/samples/etlidar_test.cpp > CMakeFiles/etlidar_test.dir/etlidar_test.cpp.i

samples/CMakeFiles/etlidar_test.dir/etlidar_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/etlidar_test.dir/etlidar_test.cpp.s"
	cd /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/samples/etlidar_test.cpp -o CMakeFiles/etlidar_test.dir/etlidar_test.cpp.s

# Object files for target etlidar_test
etlidar_test_OBJECTS = \
"CMakeFiles/etlidar_test.dir/etlidar_test.cpp.o"

# External object files for target etlidar_test
etlidar_test_EXTERNAL_OBJECTS =

etlidar_test: samples/CMakeFiles/etlidar_test.dir/etlidar_test.cpp.o
etlidar_test: samples/CMakeFiles/etlidar_test.dir/build.make
etlidar_test: libydlidar_sdk.a
etlidar_test: samples/CMakeFiles/etlidar_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../etlidar_test"
	cd /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/etlidar_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/CMakeFiles/etlidar_test.dir/build: etlidar_test

.PHONY : samples/CMakeFiles/etlidar_test.dir/build

samples/CMakeFiles/etlidar_test.dir/clean:
	cd /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples && $(CMAKE_COMMAND) -P CMakeFiles/etlidar_test.dir/cmake_clean.cmake
.PHONY : samples/CMakeFiles/etlidar_test.dir/clean

samples/CMakeFiles/etlidar_test.dir/depend:
	cd /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/samples /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples /home/yushan/Eurobot-2023/src/Eurobot-Localization/YDLidar-SDK/build/samples/CMakeFiles/etlidar_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/CMakeFiles/etlidar_test.dir/depend

