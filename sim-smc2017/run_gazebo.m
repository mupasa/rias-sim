%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research: Fault Detection Using an Extended Kalman Filter
% MATLAB-GAZEBO Simulation
% 
% ROS MASTER Configuration
%  - NODE1: GAZEBO
%  - NODE2: MATLAB
% This simulation will generate an attack alarm by following steps. 
%  - An EKF-based attack detection algorithm runs in MATLAB. 
%  - A controller sends velocity commands to the catvehicle in Gazebo.
% (ROS) represents a task done by ROS
%
% Paths for custom .world and .launch
% /catcar_ws/src/catvehicle-2.0.2/worlds/fdir.world
% /catcar_ws/src/catvehicle-2.0.2/launch/catvehicle_fdir.launch
%
% Created by Sangjun Lee
% 5/8/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;


%% (ROS)
% export ROS_IP=128.46.80.246
% export ROS_MASTER_URI=http://128.46.80.246:11311

% roscore
% roslaunch catvehicle catvehicle_fdir.launch
% roslaunch gzclient

% Echo some important topics
% rostopic echo /catvehicle/odom
% rostopic echo /catvehicle/waypoint
% rostopic echo /catvehicle/cmd_vel


%% Connect to the ROS master
% setenv('ROS_MASTER_URI','http://128.46.80.246:11311') % ROS IP
% setenv('ROS_IP','128.46.80.244') % MATLAB IP
% rosinit


%% Path Tracking Controller
% Define a set of waypoints for the desired path for the robot

% path = [0.00    0.00
%     12.00   10.00
%     20.00 15.00];

% path = [0.00    0.00
%     30.00   0.00
%     30.00   30.00
%     0.00    30.00];

path = [0 0 ; 19 19]


% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

% Assume an initial robot orientation (the robot orientation is the angle
% between the robot heading and the positive X-axis, measured
% counterclockwise).
initialOrientation = 0;

% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotCurrentLocation initialOrientation];

% Define the path following controller
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
% following controller using the  |<docid:robotics_ref.buoofp1-1 robotics.PurePursuit>|  object.
controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the
% controller
controller.Waypoints = path;

% Set the path following controller parameters. The desired linear
% velocity is set to 0.3 meters/second for this example.
controller.DesiredLinearVelocity = 1;

% The maximum angular velocity acts as a saturation limit for rotational velocity, which is
% set at 2 radians/second for this example.
controller.MaxAngularVelocity = 1;

% As a general rule, the lookahead distance should be larger than the desired
% linear velocity for a smooth path. The robot might cut corners when the
% lookahead distance is large. In contrast, a small lookahead distance can
% result in an unstable path following behavior. A value of 0.5 m was chosen
% for this example.
controller.LookaheadDistance = 3;


%% Drive the robot over the desired waypoints
% The path following controller provides input control signals for the
% robot, which the robot uses to drive itself along the desired path.
%
% Define a goal radius, which is the desired distance threshold
% between the robot's final location and the goal location. Once the robot is
% within this distance from the goal, it will stop. Also, you compute the current
% distance between the robot location and
% the goal location. This distance is continuously checked against the goal
% radius and the robot stops when this distance is less than the goal radius.
%
% Note that too small value of the goal radius may cause the robot to miss
% the goal, which may result in an unexpected behavior near the goal.
goalRadius = 0.5;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

% The |<docid:robotics_ref.buoofp1-1 controller>| object computes control commands for the robot.
% Drive the robot using these control commands until it reaches within the
% goal radius. If you are using an external simulator or a physical robot,
% then the controller outputs should be applied to the robot and a localization
% system may be required to update the pose of the robot. The controller runs at 10 Hz.
controlRate = robotics.Rate(10);

% Publish the topics
cmdvelpub = rospublisher('/catvehicle/cmd_vel', 'geometry_msgs/Twist');
cmdvelmsg = rosmessage(cmdvelpub);

% Subscribe to the topic
odomsub = rossubscriber('/catvehicle/odom');

% Increment for the while loop
i = 1;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Drive the robot by sending a message to the topic
    cmdvelmsg.Linear.X = v;
    cmdvelmsg.Angular.Z = omega;
    send(cmdvelpub,cmdvelmsg)
    
    % Receive a message from the topic
    odommsg = odomsub.receive;
    
    % Transform quternion to euler
    eularAngles = quat2eul([odommsg.Pose.Pose.Orientation.W,...
        odommsg.Pose.Pose.Orientation.X,...
        odommsg.Pose.Pose.Orientation.Y,...
        odommsg.Pose.Pose.Orientation.Z]);
    yaw = eularAngles(1);

    % Define the new pose by extracting data from the message received
    robotCurrentPose = [odommsg.Pose.Pose.Position.X ...
        odommsg.Pose.Pose.Position.Y ...
        yaw];

    robotTraj(i,:) = [odommsg.Pose.Pose.Position.X, odommsg.Pose.Pose.Position.Y];
    i = i+1;
            
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
end

%% Plots
sim_t = odommsg.Header.Stamp.Sec;

figure; % Trajectory
plot(path(:,1), path(:,2),'k--d',...
    robotTraj(:,1), robotTraj(:,2),'b')
grid on; legend('Desired Waypoints','Actual Trajectory')
xlabel('x (m)'); ylabel('y (m)');

%% Shut down the ROS network
% rosshutdown
