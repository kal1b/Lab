# CMake generated Testfile for 
# Source directory: /home/kal1b/catkin_ws/src/avoidance/safe_landing_planner
# Build directory: /home/kal1b/catkin_ws/build/safe_landing_planner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_safe_landing_planner_gtest_safe_landing_planner-test "/home/kal1b/catkin_ws/build/safe_landing_planner/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/kal1b/catkin_ws/build/safe_landing_planner/test_results/safe_landing_planner/gtest-safe_landing_planner-test.xml" "--return-code" "/home/kal1b/catkin_ws/devel/.private/safe_landing_planner/lib/safe_landing_planner/safe_landing_planner-test --gtest_output=xml:/home/kal1b/catkin_ws/build/safe_landing_planner/test_results/safe_landing_planner/gtest-safe_landing_planner-test.xml")
set_tests_properties(_ctest_safe_landing_planner_gtest_safe_landing_planner-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/kal1b/catkin_ws/src/avoidance/safe_landing_planner/CMakeLists.txt;136;catkin_add_gtest;/home/kal1b/catkin_ws/src/avoidance/safe_landing_planner/CMakeLists.txt;0;")
subdirs("gtest")
