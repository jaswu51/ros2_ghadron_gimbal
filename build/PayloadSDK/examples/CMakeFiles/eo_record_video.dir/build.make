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
include examples/CMakeFiles/eo_record_video.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include examples/CMakeFiles/eo_record_video.dir/compiler_depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/eo_record_video.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/eo_record_video.dir/flags.make

examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o: examples/CMakeFiles/eo_record_video.dir/flags.make
examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o: /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/eo_record_video.cpp
examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o: examples/CMakeFiles/eo_record_video.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o -MF CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o.d -o CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o -c /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/eo_record_video.cpp

examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eo_record_video.dir/eo_record_video.cpp.i"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/eo_record_video.cpp > CMakeFiles/eo_record_video.dir/eo_record_video.cpp.i

examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eo_record_video.dir/eo_record_video.cpp.s"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples/eo_record_video.cpp -o CMakeFiles/eo_record_video.dir/eo_record_video.cpp.s

# Object files for target eo_record_video
eo_record_video_OBJECTS = \
"CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o"

# External object files for target eo_record_video
eo_record_video_EXTERNAL_OBJECTS =

examples/eo_record_video: examples/CMakeFiles/eo_record_video.dir/eo_record_video.cpp.o
examples/eo_record_video: examples/CMakeFiles/eo_record_video.dir/build.make
examples/eo_record_video: /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/libs/x86_64/libPayloadSDK.a
examples/eo_record_video: /usr/lib/x86_64-linux-gnu/libcurl.so
examples/eo_record_video: examples/CMakeFiles/eo_record_video.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable eo_record_video"
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eo_record_video.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/eo_record_video.dir/build: examples/eo_record_video
.PHONY : examples/CMakeFiles/eo_record_video.dir/build

examples/CMakeFiles/eo_record_video.dir/clean:
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples && $(CMAKE_COMMAND) -P CMakeFiles/eo_record_video.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/eo_record_video.dir/clean

examples/CMakeFiles/eo_record_video.dir/depend:
	cd /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk /home/dtc-mrsd/Downloads/ros2_gimbal_ws/src/PayloadSdk/examples /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples /home/dtc-mrsd/Downloads/ros2_gimbal_ws/build/PayloadSDK/examples/CMakeFiles/eo_record_video.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/eo_record_video.dir/depend

