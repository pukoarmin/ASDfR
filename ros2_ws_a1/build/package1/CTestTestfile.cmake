# CMake generated Testfile for 
# Source directory: /home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1
# Build directory: /home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_showimage_cam2image__rmw_fastrtps_cpp "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/test_showimage_cam2image__rmw_fastrtps_cpp.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/launch_test/test_showimage_cam2image__rmw_fastrtps_cpp.txt" "--env" "RCL_ASSERT_RMW_ID_MATCHES=rmw_fastrtps_cpp" "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_showimage_cam2image__rmw_fastrtps_cpp_RelWithDebInfo.py" "--junit-xml=/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/test_showimage_cam2image__rmw_fastrtps_cpp.xunit.xml" "--package-name=package1")
set_tests_properties(test_showimage_cam2image__rmw_fastrtps_cpp PROPERTIES  LABELS "launch_test" TIMEOUT "30" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;82;add_launch_test;/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/call_for_each_rmw_implementation.cmake;1;testing_targets;/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/call_for_each_rmw_implementation.cmake;0;;/opt/ros/humble/share/rmw_implementation_cmake/cmake/call_for_each_rmw_implementation.cmake;64;include;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;92;call_for_each_rmw_implementation;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(copyright "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/copyright.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_copyright/copyright.txt" "--command" "/opt/ros/humble/bin/ament_copyright" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "200" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_copyright.cmake;51;ament_add_test;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;18;ament_copyright;/opt/ros/humble/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/cppcheck.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/humble/bin/ament_cppcheck" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/cppcheck.xunit.xml" "--include_dirs" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/include")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;66;ament_add_test;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;83;ament_cppcheck;/opt/ros/humble/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(cpplint "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/cpplint.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_cpplint/cpplint.txt" "--command" "/opt/ros/humble/bin/ament_cpplint" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/opt/ros/humble/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;35;ament_cpplint;/opt/ros/humble/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(flake8 "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/flake8.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_flake8/flake8.txt" "--command" "/opt/ros/humble/bin/ament_flake8" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_flake8/cmake/ament_flake8.cmake;63;ament_add_test;/opt/ros/humble/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;18;ament_flake8;/opt/ros/humble/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/lint_cmake.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/humble/bin/ament_lint_cmake" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/humble/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(pep257 "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/pep257.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_pep257/pep257.txt" "--command" "/opt/ros/humble/bin/ament_pep257" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/opt/ros/humble/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/uncrustify.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/humble/bin/ament_uncrustify" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;70;ament_add_test;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;34;ament_uncrustify;/opt/ros/humble/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/xmllint.xunit.xml" "--package-name" "package1" "--output-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/ament_xmllint/xmllint.txt" "--command" "/opt/ros/humble/bin/ament_xmllint" "--xunit-file" "/home/ram-user/Documents/ASDfR/ros2_ws_a1/build/package1/test_results/package1/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/humble/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/humble/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;102;ament_package;/home/ram-user/Documents/ASDfR/ros2_ws_a1/src/package1/CMakeLists.txt;0;")
