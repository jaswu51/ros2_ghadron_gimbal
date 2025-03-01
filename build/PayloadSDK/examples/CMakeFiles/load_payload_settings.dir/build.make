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
include examples/CMakeFiles/load_payload_settings.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/load_payload_settings.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/load_payload_settings.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/load_payload_settings.dir/flags.make

examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o: examples/CMakeFiles/load_payload_settings.dir/flags.make
examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o: /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/load_payload_settings.cpp
examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o: examples/CMakeFiles/load_payload_settings.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o -MF CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o.d -o CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o -c /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/load_payload_settings.cpp

examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.i"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/load_payload_settings.cpp > CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.i

examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.s"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/load_payload_settings.cpp -o CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.s

# Object files for target load_payload_settings
load_payload_settings_OBJECTS = \
"CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o"

# External object files for target load_payload_settings
load_payload_settings_EXTERNAL_OBJECTS =

examples/load_payload_settings: examples/CMakeFiles/load_payload_settings.dir/load_payload_settings.cpp.o
examples/load_payload_settings: examples/CMakeFiles/load_payload_settings.dir/build.make
examples/load_payload_settings: /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/libs/x86_64/libPayloadSDK.a
examples/load_payload_settings: /usr/lib/x86_64-linux-gnu/libcurl.so
examples/load_payload_settings: examples/CMakeFiles/load_payload_settings.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable load_payload_settings"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/load_payload_settings.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/load_payload_settings.dir/build: examples/load_payload_settings
.PHONY : examples/CMakeFiles/load_payload_settings.dir/build

examples/CMakeFiles/load_payload_settings.dir/clean:
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && $(CMAKE_COMMAND) -P CMakeFiles/load_payload_settings.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/load_payload_settings.dir/clean

examples/CMakeFiles/load_payload_settings.dir/depend:
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples/CMakeFiles/load_payload_settings.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/load_payload_settings.dir/depend

