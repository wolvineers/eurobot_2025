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
include modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/depend.make

# Include the progress variables for this target.
include modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/progress.make

# Include the compile flags for this target's objects.
include modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/flags.make

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.o: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/flags.make
modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/feature_evaluator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/feature_evaluator.cpp

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/feature_evaluator.cpp > CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.i

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/feature_evaluator.cpp -o CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.s

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.o: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/flags.make
modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/lbpfeatures.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/lbpfeatures.cpp

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/lbpfeatures.cpp > CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.i

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/lbpfeatures.cpp -o CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.s

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.o: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/flags.make
modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/waldboost.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/waldboost.cpp

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/waldboost.cpp > CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.i

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/waldboost.cpp -o CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.s

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.o: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/flags.make
modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.o: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/wbdetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.o"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.o -c /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/wbdetector.cpp

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.i"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/wbdetector.cpp > CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.i

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.s"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect/src/wbdetector.cpp -o CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.s

# Object files for target opencv_xobjdetect
opencv_xobjdetect_OBJECTS = \
"CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.o" \
"CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.o" \
"CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.o" \
"CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.o"

# External object files for target opencv_xobjdetect
opencv_xobjdetect_EXTERNAL_OBJECTS =

lib/libopencv_xobjdetect.so.4.10.0: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/feature_evaluator.cpp.o
lib/libopencv_xobjdetect.so.4.10.0: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/lbpfeatures.cpp.o
lib/libopencv_xobjdetect.so.4.10.0: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/waldboost.cpp.o
lib/libopencv_xobjdetect.so.4.10.0: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/src/wbdetector.cpp.o
lib/libopencv_xobjdetect.so.4.10.0: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/build.make
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_imgcodecs.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_objdetect.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: 3rdparty/lib/libippiw.a
lib/libopencv_xobjdetect.so.4.10.0: 3rdparty/ippicv/ippicv_lnx/icv/lib/intel64/libippicv.a
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_dnn.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_calib3d.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_features2d.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_flann.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_imgproc.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: lib/libopencv_core.so.4.10.0
lib/libopencv_xobjdetect.so.4.10.0: modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library ../../lib/libopencv_xobjdetect.so"
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_xobjdetect.dir/link.txt --verbose=$(VERBOSE)
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && $(CMAKE_COMMAND) -E cmake_symlink_library ../../lib/libopencv_xobjdetect.so.4.10.0 ../../lib/libopencv_xobjdetect.so.410 ../../lib/libopencv_xobjdetect.so

lib/libopencv_xobjdetect.so.410: lib/libopencv_xobjdetect.so.4.10.0
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libopencv_xobjdetect.so.410

lib/libopencv_xobjdetect.so: lib/libopencv_xobjdetect.so.4.10.0
	@$(CMAKE_COMMAND) -E touch_nocreate lib/libopencv_xobjdetect.so

# Rule to build all files generated by this target.
modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/build: lib/libopencv_xobjdetect.so

.PHONY : modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/build

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/clean:
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect && $(CMAKE_COMMAND) -P CMakeFiles/opencv_xobjdetect.dir/cmake_clean.cmake
.PHONY : modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/clean

modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/depend:
	cd /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/xobjdetect /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : modules/xobjdetect/CMakeFiles/opencv_xobjdetect.dir/depend

