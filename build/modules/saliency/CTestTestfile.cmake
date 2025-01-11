# CMake generated Testfile for 
# Source directory: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/saliency
# Build directory: /home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/modules/saliency
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(opencv_test_saliency "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/bin/opencv_test_saliency" "--gtest_output=xml:opencv_test_saliency.xml")
set_tests_properties(opencv_test_saliency PROPERTIES  LABELS "Extra;opencv_saliency;Accuracy" WORKING_DIRECTORY "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/build/test-reports/accuracy" _BACKTRACE_TRIPLES "/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/cmake/OpenCVUtils.cmake;1799;add_test;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/cmake/OpenCVModule.cmake;1365;ocv_add_test_from_target;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv-4.x/cmake/OpenCVModule.cmake;1123;ocv_add_accuracy_tests;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/saliency/CMakeLists.txt;7;ocv_define_module;/home/aida/Desktop/robotica/eurobot_2025/eurobot_2025/opencv_contrib-4.x/modules/saliency/CMakeLists.txt;0;")
