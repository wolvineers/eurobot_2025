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
CMAKE_SOURCE_DIR = /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build

# Include any dependencies generated for this target.
include modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/depend.make

# Include the progress variables for this target.
include modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/progress.make

# Include the compile flags for this target's objects.
include modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/flags.make

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.o: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/flags.make
modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_faps.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_faps.cpp

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_faps.cpp > CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.i

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_faps.cpp -o CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.s

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.o: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/flags.make
modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_getProjPixel.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_getProjPixel.cpp

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_getProjPixel.cpp > CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.i

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_getProjPixel.cpp -o CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.s

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.o: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/flags.make
modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_main.cpp

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_main.cpp > CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.i

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_main.cpp -o CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.s

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.o: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/flags.make
modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_plane.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_plane.cpp

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_plane.cpp > CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.i

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light/test/test_plane.cpp -o CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.s

# Object files for target opencv_test_structured_light
opencv_test_structured_light_OBJECTS = \
"CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.o" \
"CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.o" \
"CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.o" \
"CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.o"

# External object files for target opencv_test_structured_light
opencv_test_structured_light_EXTERNAL_OBJECTS =

bin/opencv_test_structured_light: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_faps.cpp.o
bin/opencv_test_structured_light: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_getProjPixel.cpp.o
bin/opencv_test_structured_light: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_main.cpp.o
bin/opencv_test_structured_light: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/test/test_plane.cpp.o
bin/opencv_test_structured_light: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/build.make
bin/opencv_test_structured_light: lib/libopencv_ts.a
bin/opencv_test_structured_light: lib/libopencv_structured_light.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_highgui.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_phase_unwrapping.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_viz.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_calib3d.so.4.10.0
bin/opencv_test_structured_light: 3rdparty/lib/libippiw.a
bin/opencv_test_structured_light: 3rdparty/ippicv/ippicv_lnx/icv/lib/intel64/libippicv.a
bin/opencv_test_structured_light: lib/libopencv_videoio.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_imgcodecs.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_features2d.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_flann.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_imgproc.so.4.10.0
bin/opencv_test_structured_light: lib/libopencv_core.so.4.10.0
bin/opencv_test_structured_light: modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable ../../bin/opencv_test_structured_light"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_test_structured_light.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/build: bin/opencv_test_structured_light

.PHONY : modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/build

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/clean:
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light && $(CMAKE_COMMAND) -P CMakeFiles/opencv_test_structured_light.dir/cmake_clean.cmake
.PHONY : modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/clean

modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/depend:
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/structured_light /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/structured_light/CMakeFiles/opencv_test_structured_light.dir/depend

