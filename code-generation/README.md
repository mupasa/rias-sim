# README.md
This tutorial is to demonstrate how to generate code from Simulink that will run in your ROS workspace. 
For more details, see https://www.mathworks.com/help/robotics/examples/generate-a-standalone-ros-node-in-simulink.html

## MATLAB Device
### Task 1. Configure a Model for Code Generation
In the Hardware Implementation pane of the Model Configuration Parameters dialog, set Hardware board to Robot Operating System (ROS).

### Task 2. Configure the Connection to the ROS Device
In the Target Hardware Resouces tab of the Hardware board settings, set Device Parameters to the target device's IP address and username, an appropriate ROS folder and Catkin workspace (eg. /opt/ros/kinetic and /home/username/catkin_ws).

The selected Build action affects the behavior of Simulink when building the model. 
* None (the default setting) only generates the code for the ROS node, without building it on an external ROS device. 
* Build and load generates the code, transfers it to an external device and builds a ROS node executable. 
* Build and run generates the resulting node executable is started automatically at the end of the build.

If the ROS device is turned on and accessible from your computer (if roscore is running), you can verify the connection settings by clicking Test.

Click Apply if everything runs fine.

### Task 3. Generate the C++ ROS Node
Click on Code > C/C++ Code > Deploy to Hardware. If you get any errors about bus type mismatch, close the model, clear all variables from the base MATLAB workspace, and re-open the model.

Once the code generation completes, the ROS node is transferred to the Catkin workspace on your ROS device.

Generated C++ code archive: No matter what Build action you select (None, Build and load, Build and run), Simulink always generates two files in your current folder: an archive containing the C++ source code (RobotController.tgz in our example) and a shell script for extracting and building the C++ code manually (build_ros_model.sh). If your MATLAB computer is not connected to the ROS device, you can transfer the files manually and build them there.

## ROS Device
### Task 1. Verify the Node
Go to the Catkin worksapce you created ("/home/username/catkin_ws" in this tutorial) and see if the ROS node was successfully generated.

### Task 2. Run the Node
Ready to start up the simulation, and run the this tutorial from the generated code, rather than from SimulinkIf. Add the .bash to the source.

    source devel/setup.bash

and run the node.

    rosrun exampletutorial exampletutorial_node
