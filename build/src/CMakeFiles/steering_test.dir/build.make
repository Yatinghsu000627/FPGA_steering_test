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
CMAKE_SOURCE_DIR = /home/tina997726/corgi_ws/steering_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tina997726/corgi_ws/steering_test/build

# Include any dependencies generated for this target.
include src/CMakeFiles/steering_test.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/steering_test.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/steering_test.dir/flags.make

src/CMakeFiles/steering_test.dir/steering_test.cpp.o: src/CMakeFiles/steering_test.dir/flags.make
src/CMakeFiles/steering_test.dir/steering_test.cpp.o: ../src/steering_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tina997726/corgi_ws/steering_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/steering_test.dir/steering_test.cpp.o"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/steering_test.dir/steering_test.cpp.o -c /home/tina997726/corgi_ws/steering_test/src/steering_test.cpp

src/CMakeFiles/steering_test.dir/steering_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/steering_test.dir/steering_test.cpp.i"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tina997726/corgi_ws/steering_test/src/steering_test.cpp > CMakeFiles/steering_test.dir/steering_test.cpp.i

src/CMakeFiles/steering_test.dir/steering_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/steering_test.dir/steering_test.cpp.s"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tina997726/corgi_ws/steering_test/src/steering_test.cpp -o CMakeFiles/steering_test.dir/steering_test.cpp.s

src/CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.o: src/CMakeFiles/steering_test.dir/flags.make
src/CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.o: ../src/NiFpga_FPGA_CANBus_4module_v3_steering.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tina997726/corgi_ws/steering_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.o"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.o   -c /home/tina997726/corgi_ws/steering_test/src/NiFpga_FPGA_CANBus_4module_v3_steering.c

src/CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.i"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tina997726/corgi_ws/steering_test/src/NiFpga_FPGA_CANBus_4module_v3_steering.c > CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.i

src/CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.s"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tina997726/corgi_ws/steering_test/src/NiFpga_FPGA_CANBus_4module_v3_steering.c -o CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.s

src/CMakeFiles/steering_test.dir/NiFpga.c.o: src/CMakeFiles/steering_test.dir/flags.make
src/CMakeFiles/steering_test.dir/NiFpga.c.o: ../src/NiFpga.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tina997726/corgi_ws/steering_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object src/CMakeFiles/steering_test.dir/NiFpga.c.o"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/steering_test.dir/NiFpga.c.o   -c /home/tina997726/corgi_ws/steering_test/src/NiFpga.c

src/CMakeFiles/steering_test.dir/NiFpga.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/steering_test.dir/NiFpga.c.i"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/tina997726/corgi_ws/steering_test/src/NiFpga.c > CMakeFiles/steering_test.dir/NiFpga.c.i

src/CMakeFiles/steering_test.dir/NiFpga.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/steering_test.dir/NiFpga.c.s"
	cd /home/tina997726/corgi_ws/steering_test/build/src && /usr/bin/gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/tina997726/corgi_ws/steering_test/src/NiFpga.c -o CMakeFiles/steering_test.dir/NiFpga.c.s

# Object files for target steering_test
steering_test_OBJECTS = \
"CMakeFiles/steering_test.dir/steering_test.cpp.o" \
"CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.o" \
"CMakeFiles/steering_test.dir/NiFpga.c.o"

# External object files for target steering_test
steering_test_EXTERNAL_OBJECTS =

steering_test: src/CMakeFiles/steering_test.dir/steering_test.cpp.o
steering_test: src/CMakeFiles/steering_test.dir/NiFpga_FPGA_CANBus_4module_v3_steering.c.o
steering_test: src/CMakeFiles/steering_test.dir/NiFpga.c.o
steering_test: src/CMakeFiles/steering_test.dir/build.make
steering_test: src/CMakeFiles/steering_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tina997726/corgi_ws/steering_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../steering_test"
	cd /home/tina997726/corgi_ws/steering_test/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/steering_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/steering_test.dir/build: steering_test

.PHONY : src/CMakeFiles/steering_test.dir/build

src/CMakeFiles/steering_test.dir/clean:
	cd /home/tina997726/corgi_ws/steering_test/build/src && $(CMAKE_COMMAND) -P CMakeFiles/steering_test.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/steering_test.dir/clean

src/CMakeFiles/steering_test.dir/depend:
	cd /home/tina997726/corgi_ws/steering_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tina997726/corgi_ws/steering_test /home/tina997726/corgi_ws/steering_test/src /home/tina997726/corgi_ws/steering_test/build /home/tina997726/corgi_ws/steering_test/build/src /home/tina997726/corgi_ws/steering_test/build/src/CMakeFiles/steering_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/steering_test.dir/depend

