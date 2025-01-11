# CMake generated Testfile for 
# Source directory: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/modules/highgui
# Build directory: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/highgui
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_highgui "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/bin/opencv_test_highgui" "--gtest_output=xml:opencv_test_highgui.xml")
set_tests_properties(opencv_test_highgui PROPERTIES  LABELS "Main;opencv_highgui;Accuracy" WORKING_DIRECTORY "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/cmake/OpenCVModule.cmake;1365;ocv_add_test_from_target;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/modules/highgui/CMakeLists.txt;310;ocv_add_accuracy_tests;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/modules/highgui/CMakeLists.txt;0;")
