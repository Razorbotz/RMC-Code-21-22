# CMake generated Testfile for 
# Source directory: /home/team/SoftwareDevelopment/ROS2/skinny/src/messages
# Build directory: /home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/eloquent/share/ament_cmake_test/cmake/run_test.py" "/home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages/test_results/messages/lint_cmake.xunit.xml" "--package-name" "messages" "--output-file" "/home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/eloquent/bin/ament_lint_cmake" "--xunit-file" "/home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages/test_results/messages/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/team/SoftwareDevelopment/ROS2/skinny/src/messages")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/eloquent/share/ament_cmake_test/cmake/run_test.py" "/home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages/test_results/messages/xmllint.xunit.xml" "--package-name" "messages" "--output-file" "/home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages/ament_xmllint/xmllint.txt" "--command" "/opt/ros/eloquent/bin/ament_xmllint" "--xunit-file" "/home/team/SoftwareDevelopment/ROS2/skinny/src/build/messages/test_results/messages/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/team/SoftwareDevelopment/ROS2/skinny/src/messages")
subdirs("messages__py")
