Download all the packages (unit_test, jiwy_simulator, package1) in the source directory of your ros2 workspace (example ros2_ws/src)
Navigate to the workspace directory ros2_ws 
colcon build
Open new terminal
cd ros2_ws
. install/setup.bash
cd src/unit_test/launch
ros2 launch assignment12x.py ( x = 1 for assignment 1.2.1
                                x = 2 for assignment 1.2.2
                                x = 3 for assignment 1.2.3)

The launch files will run all the required nodes
Make sure that the webcam is already selected from the drop down menu item Devices | Webcams of the Virtualbox window.

The webcam input and the moving camera output can be viewed in rqt
Open new terminal
cd ros2_ws
. install/setup.bash
rqt 
From the drop down menu, select Plugins-> Visulization -> Image View (sometimes you need to refresh and then select the topic)

