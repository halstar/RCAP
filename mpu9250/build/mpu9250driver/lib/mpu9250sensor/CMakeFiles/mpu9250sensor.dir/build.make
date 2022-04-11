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
CMAKE_SOURCE_DIR = /home/stef/RCAP/mpu9250

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stef/RCAP/mpu9250/build/mpu9250driver

# Include any dependencies generated for this target.
include lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/depend.make

# Include the progress variables for this target.
include lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/progress.make

# Include the compile flags for this target's objects.
include lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/flags.make

lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.o: lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/flags.make
lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.o: ../../lib/mpu9250sensor/src/mpu9250sensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stef/RCAP/mpu9250/build/mpu9250driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.o"
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.o -c /home/stef/RCAP/mpu9250/lib/mpu9250sensor/src/mpu9250sensor.cpp

lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.i"
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stef/RCAP/mpu9250/lib/mpu9250sensor/src/mpu9250sensor.cpp > CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.i

lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.s"
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stef/RCAP/mpu9250/lib/mpu9250sensor/src/mpu9250sensor.cpp -o CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.s

# Object files for target mpu9250sensor
mpu9250sensor_OBJECTS = \
"CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.o"

# External object files for target mpu9250sensor
mpu9250sensor_EXTERNAL_OBJECTS =

lib/mpu9250sensor/libmpu9250sensor.a: lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/src/mpu9250sensor.cpp.o
lib/mpu9250sensor/libmpu9250sensor.a: lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/build.make
lib/mpu9250sensor/libmpu9250sensor.a: lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/stef/RCAP/mpu9250/build/mpu9250driver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmpu9250sensor.a"
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor && $(CMAKE_COMMAND) -P CMakeFiles/mpu9250sensor.dir/cmake_clean_target.cmake
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpu9250sensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/build: lib/mpu9250sensor/libmpu9250sensor.a

.PHONY : lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/build

lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/clean:
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor && $(CMAKE_COMMAND) -P CMakeFiles/mpu9250sensor.dir/cmake_clean.cmake
.PHONY : lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/clean

lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/depend:
	cd /home/stef/RCAP/mpu9250/build/mpu9250driver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stef/RCAP/mpu9250 /home/stef/RCAP/mpu9250/lib/mpu9250sensor /home/stef/RCAP/mpu9250/build/mpu9250driver /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor /home/stef/RCAP/mpu9250/build/mpu9250driver/lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/mpu9250sensor/CMakeFiles/mpu9250sensor.dir/depend

