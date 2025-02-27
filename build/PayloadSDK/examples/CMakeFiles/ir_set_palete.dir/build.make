# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK

# Include any dependencies generated for this target.
include examples/CMakeFiles/ir_set_palete.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/ir_set_palete.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/ir_set_palete.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/ir_set_palete.dir/flags.make

examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o: examples/CMakeFiles/ir_set_palete.dir/flags.make
examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o: /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/ir_set_palete.cpp
examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o: examples/CMakeFiles/ir_set_palete.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o -MF CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o.d -o CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o -c /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/ir_set_palete.cpp

examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.i"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/ir_set_palete.cpp > CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.i

examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.s"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/ir_set_palete.cpp -o CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.s

# Object files for target ir_set_palete
ir_set_palete_OBJECTS = \
"CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o"

# External object files for target ir_set_palete
ir_set_palete_EXTERNAL_OBJECTS =

examples/ir_set_palete: examples/CMakeFiles/ir_set_palete.dir/ir_set_palete.cpp.o
examples/ir_set_palete: examples/CMakeFiles/ir_set_palete.dir/build.make
examples/ir_set_palete: /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/libs/x86_64/libPayloadSDK.a
examples/ir_set_palete: /usr/lib/x86_64-linux-gnu/libcurl.so
examples/ir_set_palete: examples/CMakeFiles/ir_set_palete.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ir_set_palete"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ir_set_palete.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/ir_set_palete.dir/build: examples/ir_set_palete
.PHONY : examples/CMakeFiles/ir_set_palete.dir/build

examples/CMakeFiles/ir_set_palete.dir/clean:
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && $(CMAKE_COMMAND) -P CMakeFiles/ir_set_palete.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/ir_set_palete.dir/clean

examples/CMakeFiles/ir_set_palete.dir/depend:
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples/CMakeFiles/ir_set_palete.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/ir_set_palete.dir/depend

