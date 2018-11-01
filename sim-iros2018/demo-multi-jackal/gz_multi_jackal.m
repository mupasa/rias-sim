%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research
% MATLAB-GAZEBO Multi-Robot Simulation
% 
% MATLAB Configuration
%  - Run the controllers
%
% ROS Nodes Configuration
%  - Run GAZEBO simulator
%
% Useful Topics
%  - Control input: '/jackal0/jackal_velocity_controller/cmd_vel'
%  - Output: '/jackal0//jackal_velocity_controller/odom'
%
% Change intiail locations of robots
%  /ops/ros/kinetic/share/multi_jackal_tutorials/launch/custom_multi_jackal.launch
%
% Created by Sangjun Lee
% 9/27/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc; 
%tic;

%% Running Gazebo
% roslaunch multi_jackal_tutorials custom_multi_jackal.launch
% gzclient

%% Connect to the ROS master
% setenv('ROS_MASTER_URI','http://192.168.1.6:11311') % ROS MASTER IP
% setenv('ROS_IP','192.168.1.3') % MATLAB IP
% rosinit

%% Simulation Design Variables

% State vector.
%disp('State Vector: x = [x y theta vx vy thetadot ax ay bthetadot bax bay]')

% Set the sampling time (sec)
Ts = 0.01;  % 100 Hz

% Attack Profile
att_t = 20;            % Injection time (sec)
att_state = 2;         % Target state

fprintf('Attack Profile: %d sec, %d y-axis',att_t,att_state)

% Define the controller rate
controlRate = robotics.Rate(1/Ts); % Hz

% Set the goal location of the robots
rdv = [0 0]; % Rendezvous point
r0_Goal = rdv; % r0 initial location
r1_Goal = rdv; % r1 initial location
r2_Goal = rdv; % r2 initial location
r3_Goal = rdv; % r3 initial location
r4_Goal = rdv; % r4 initial location
r5_Goal = rdv; % r5 initial location
r6_Goal = rdv; % r6 initial location
r7_Goal = rdv; % r7 initial location
r8_Goal = rdv; % r8 initial location

fprintf('Rendezvous at (%d, %d)\n',rdv(1),rdv(2))

% Set the contoller specifications
DesiredLinearVelocity = 1; % 1m/sec = 3.6km/h
MaxAngularVelocity = 0.5; % rad/sec
LookaheadDistance = 5; % m
goalRadius = 1.4; % m


%% Define ROS Topic Publisher and Subscribers

% Publish topics as control inputs
r0_cmdvelpub = rospublisher('/jackal0/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r0_cmdvelmsg = rosmessage(r0_cmdvelpub);
r1_cmdvelpub = rospublisher('/jackal1/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r1_cmdvelmsg = rosmessage(r1_cmdvelpub);
r2_cmdvelpub = rospublisher('/jackal2/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r2_cmdvelmsg = rosmessage(r2_cmdvelpub);
r3_cmdvelpub = rospublisher('/jackal3/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r3_cmdvelmsg = rosmessage(r3_cmdvelpub);
r4_cmdvelpub = rospublisher('/jackal4/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r4_cmdvelmsg = rosmessage(r4_cmdvelpub);
r5_cmdvelpub = rospublisher('/jackal5/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r5_cmdvelmsg = rosmessage(r5_cmdvelpub);
r6_cmdvelpub = rospublisher('/jackal6/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r6_cmdvelmsg = rosmessage(r6_cmdvelpub);
r7_cmdvelpub = rospublisher('/jackal7/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r7_cmdvelmsg = rosmessage(r7_cmdvelpub);
r8_cmdvelpub = rospublisher('/jackal8/jackal_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r8_cmdvelmsg = rosmessage(r8_cmdvelpub);

% Subscribe topics as feedback
r0_odomsub = rossubscriber('/jackal0/odometry/global_filtered');
r1_odomsub = rossubscriber('/jackal1/odometry/global_filtered');
r2_odomsub = rossubscriber('/jackal2/odometry/global_filtered');
r3_odomsub = rossubscriber('/jackal3/odometry/global_filtered');
r4_odomsub = rossubscriber('/jackal4/odometry/global_filtered');
r5_odomsub = rossubscriber('/jackal5/odometry/global_filtered');
r6_odomsub = rossubscriber('/jackal6/odometry/global_filtered');
r7_odomsub = rossubscriber('/jackal7/odometry/global_filtered');
r8_odomsub = rossubscriber('/jackal8/odometry/global_filtered');


%% Controller Design for ROBOT 0

% Set the current location of the robot
r0_initmsg = r0_odomsub.receive;
r0_robotCurrentLocation = [r0_initmsg.Pose.Pose.Position.X,...
    r0_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r0_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r0_robotCurrentPose = [r0_robotCurrentLocation r0_initialOrientation];

% Define the path following controller
r0_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r0_controller.Waypoints = [r0_robotCurrentLocation ; r0_Goal];

% Set the path following controller parameters.
r0_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r0_controller.MaxAngularVelocity = MaxAngularVelocity;
r0_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 1

% Set the current location of the robot
r1_initmsg = r1_odomsub.receive;
r1_robotCurrentLocation = [r1_initmsg.Pose.Pose.Position.X,...
    r1_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r1_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r1_robotCurrentPose = [r1_robotCurrentLocation r1_initialOrientation];

% Define the path following controller
r1_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r1_controller.Waypoints = [r1_robotCurrentLocation ; r1_Goal];

% Set the path following controller parameters.
r1_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r1_controller.MaxAngularVelocity = MaxAngularVelocity;
r1_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 2

% Set the current location of the robot
r2_initmsg = r2_odomsub.receive;
r2_robotCurrentLocation = [r2_initmsg.Pose.Pose.Position.X,...
    r2_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r2_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r2_robotCurrentPose = [r2_robotCurrentLocation r2_initialOrientation];

% Define the path following controller
r2_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r2_controller.Waypoints = [r2_robotCurrentLocation ; r2_Goal];

% Set the path following controller parameters.
r2_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r2_controller.MaxAngularVelocity = MaxAngularVelocity;
r2_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 3

% Set the current location of the robot
%r3_initmsg = r3_odomsub.receive;
r3_robotCurrentLocation = [0, 0];

% Set an initial robot orientation 
r3_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r3_robotCurrentPose = [r3_robotCurrentLocation r3_initialOrientation];

% Define the path following controller
r3_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r3_controller.Waypoints = [r3_robotCurrentLocation ; r3_Goal];

% Set the path following controller parameters.
r3_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r3_controller.MaxAngularVelocity = MaxAngularVelocity;
r3_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 4

% Set the current location of the robot
r4_initmsg = r4_odomsub.receive;
r4_robotCurrentLocation = [r4_initmsg.Pose.Pose.Position.X,...
    r4_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r4_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r4_robotCurrentPose = [r4_robotCurrentLocation r4_initialOrientation];

% Define the path following controller
r4_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r4_controller.Waypoints = [r4_robotCurrentLocation ; r4_Goal];

% Set the path following controller parameters.
r4_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r4_controller.MaxAngularVelocity = MaxAngularVelocity;
r4_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 5

% Set the current location of the robot
r5_initmsg = r5_odomsub.receive;
r5_robotCurrentLocation = [r5_initmsg.Pose.Pose.Position.X,...
    r5_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r5_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r5_robotCurrentPose = [r5_robotCurrentLocation r5_initialOrientation];

% Define the path following controller
r5_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r5_controller.Waypoints = [r5_robotCurrentLocation ; r5_Goal];

% Set the path following controller parameters.
r5_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r5_controller.MaxAngularVelocity = MaxAngularVelocity;
r5_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 6

% Set the current location of the robot
r6_initmsg = r6_odomsub.receive;
r6_robotCurrentLocation = [r6_initmsg.Pose.Pose.Position.X,...
    r6_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r6_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r6_robotCurrentPose = [r6_robotCurrentLocation r6_initialOrientation];

% Define the path following controller
r6_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r6_controller.Waypoints = [r6_robotCurrentLocation ; r6_Goal];

% Set the path following controller parameters.
r6_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r6_controller.MaxAngularVelocity = MaxAngularVelocity;
r6_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 7

% Set the current location of the robot
r7_initmsg = r7_odomsub.receive;
r7_robotCurrentLocation = [r7_initmsg.Pose.Pose.Position.X,...
    r7_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r7_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r7_robotCurrentPose = [r7_robotCurrentLocation r7_initialOrientation];

% Define the path following controller
r7_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r7_controller.Waypoints = [r7_robotCurrentLocation ; r7_Goal];

% Set the path following controller parameters.
r7_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r7_controller.MaxAngularVelocity = MaxAngularVelocity;
r7_controller.LookaheadDistance = LookaheadDistance;


%% Controller Design for ROBOT 8

% Set the current location of the robot
r8_initmsg = r8_odomsub.receive;
r8_robotCurrentLocation = [r8_initmsg.Pose.Pose.Position.X,...
    r8_initmsg.Pose.Pose.Position.Y];

% Set an initial robot orientation 
r8_initialOrientation = 0;

% Define the current pose for the robot [x y theta]
r8_robotCurrentPose = [r8_robotCurrentLocation r8_initialOrientation];

% Define the path following controller
r8_controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
r8_controller.Waypoints = [r8_robotCurrentLocation ; r8_Goal];

% Set the path following controller parameters.
r8_controller.DesiredLinearVelocity = DesiredLinearVelocity;
r8_controller.MaxAngularVelocity = MaxAngularVelocity;
r8_controller.LookaheadDistance = LookaheadDistance;


%% Simulation

tEnd = 60;                    % Final time (sec)
t  = 0:Ts:tEnd;                % Simulation time with Ts sampling period
tFault = zeros(1);             % Alarm time (sec)

for ct = 1:numel(t) % Main Loop
%% Feedback Control for ROBOT 0
    % Compute the controller outputs, i.e., the inputs to the robot
    [r0_v, r0_omega] = r0_controller(r0_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r0_distanceToGoal = norm(r0_robotCurrentPose(1:2) - r0_Goal);
    if (r0_distanceToGoal < goalRadius)
        % If the robot reaches within the goal radius, STOP
        r0_cmdvelmsg.Linear.X = 0;
        r0_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r0_cmdvelmsg.Linear.X = r0_v;
        r0_cmdvelmsg.Angular.Z = r0_omega;
    end
    
    % Send the message
    send(r0_cmdvelpub,r0_cmdvelmsg)
    
    % Receive a message from the topic
    r0_odommsg = r0_odomsub.receive;
    
    % Transform quternion to euler
    r0_eularAngles = quat2eul([r0_odommsg.Pose.Pose.Orientation.W,...
        r0_odommsg.Pose.Pose.Orientation.X,...
        r0_odommsg.Pose.Pose.Orientation.Y,...
        r0_odommsg.Pose.Pose.Orientation.Z]);
    r0_yaw = r0_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r0_robotCurrentPose = [r0_odommsg.Pose.Pose.Position.X ...
        r0_odommsg.Pose.Pose.Position.Y ...
        r0_yaw];
    r0_traj(ct,:) = r0_robotCurrentPose;
    
    
    %% Feedback Control for ROBOT 1
    % Compute the controller outputs, i.e., the inputs to the robot
    [r1_v, r1_omega] = r1_controller(r1_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r1_distanceToGoal = norm(r1_robotCurrentPose(1:2) - r1_Goal);
    if (r1_distanceToGoal < goalRadius)
        % If the robot reaches within the goal radius, STOP
        r1_cmdvelmsg.Linear.X = 0;
        r1_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r1_cmdvelmsg.Linear.X = r1_v;
        r1_cmdvelmsg.Angular.Z = r1_omega;
    end
    
    % Send the message
    send(r1_cmdvelpub,r1_cmdvelmsg)
    
    % Receive a message from the topic
    r1_odommsg = r1_odomsub.receive;
    
    % Transform quternion to euler
    r1_eularAngles = quat2eul([r1_odommsg.Pose.Pose.Orientation.W,...
        r1_odommsg.Pose.Pose.Orientation.X,...
        r1_odommsg.Pose.Pose.Orientation.Y,...
        r1_odommsg.Pose.Pose.Orientation.Z]);
    r1_yaw = r1_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r1_robotCurrentPose = [r1_odommsg.Pose.Pose.Position.X ...
        r1_odommsg.Pose.Pose.Position.Y ...
        r1_yaw];
    r1_traj(ct,:) = r1_robotCurrentPose;
    

%% Feedback Control for ROBOT 2
    % Compute the controller outputs, i.e., the inputs to the robot
    [r2_v, r2_omega] = r2_controller(r2_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r2_distanceToGoal = norm(r2_robotCurrentPose(1:2) - r2_Goal);
    if (r2_distanceToGoal < goalRadius)
        % If the robot reaches within the goal radius, STOP
        r2_cmdvelmsg.Linear.X = 0;
        r2_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r2_cmdvelmsg.Linear.X = r2_v;
        r2_cmdvelmsg.Angular.Z = r2_omega;
    end
    
    % Send the message
    send(r2_cmdvelpub,r2_cmdvelmsg)
    
    % Receive a message from the topic
    r2_odommsg = r2_odomsub.receive;
    
    % Transform quternion to euler
    r2_eularAngles = quat2eul([r2_odommsg.Pose.Pose.Orientation.W,...
        r2_odommsg.Pose.Pose.Orientation.X,...
        r2_odommsg.Pose.Pose.Orientation.Y,...
        r2_odommsg.Pose.Pose.Orientation.Z]);
    r2_yaw = r2_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r2_robotCurrentPose = [r2_odommsg.Pose.Pose.Position.X ...
        r2_odommsg.Pose.Pose.Position.Y ...
        r2_yaw];
    r2_traj(ct,:) = r2_robotCurrentPose;
    
%% Feedback Control for ROBOT 3
    % Compute the controller outputs, i.e., the inputs to the robot
    [r3_v, r3_omega] = r3_controller(r3_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r3_distanceToGoal = norm(r3_robotCurrentPose(1:2) - r3_Goal);
    if (r3_distanceToGoal < goalRadius) || (tFault ~= 0)
        % If the robot reaches within the goal radius, STOP
        r3_cmdvelmsg.Linear.X = 0;
        r3_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r3_cmdvelmsg.Linear.X = r3_v;
        r3_cmdvelmsg.Angular.Z = r3_omega;
    end
    
    % Send the message
    send(r3_cmdvelpub,r3_cmdvelmsg)
    
    % Receive a message from the topic
    r3_odommsg = r3_odomsub.receive;
    
    % Transform quternion to euler
    r3_eularAngles = quat2eul([r3_odommsg.Pose.Pose.Orientation.W,...
        r3_odommsg.Pose.Pose.Orientation.X,...
        r3_odommsg.Pose.Pose.Orientation.Y,...
        r3_odommsg.Pose.Pose.Orientation.Z]);
    r3_yaw = r3_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r3_robotCurrentPose = [r3_odommsg.Pose.Pose.Position.X ...
        r3_odommsg.Pose.Pose.Position.Y ...
        r3_yaw];
    r3_traj(ct,:) = r3_robotCurrentPose;
    
    
%% Feedback Control for ROBOT 4
    % Compute the controller outputs, i.e., the inputs to the robot
    [r4_v, r4_omega] = r4_controller(r4_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r4_distanceToGoal = norm(r4_robotCurrentPose(1:2) - r4_Goal);
    if (r4_distanceToGoal < goalRadius) || (tFault ~= 0)
        % If the robot reaches within the goal radius, STOP
        r4_cmdvelmsg.Linear.X = 0;
        r4_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r4_cmdvelmsg.Linear.X = r4_v;
        r4_cmdvelmsg.Angular.Z = r4_omega;
    end
    
    % Send the message
    send(r4_cmdvelpub,r4_cmdvelmsg)
    
    % Receive a message from the topic
    r4_odommsg = r4_odomsub.receive;
    
    % Transform quternion to euler
    r4_eularAngles = quat2eul([r4_odommsg.Pose.Pose.Orientation.W,...
        r4_odommsg.Pose.Pose.Orientation.X,...
        r4_odommsg.Pose.Pose.Orientation.Y,...
        r4_odommsg.Pose.Pose.Orientation.Z]);
    r4_yaw = r4_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r4_robotCurrentPose = [r4_odommsg.Pose.Pose.Position.X ...
        r4_odommsg.Pose.Pose.Position.Y ...
        r4_yaw];
    r4_traj(ct,:) = r4_robotCurrentPose;
    
 
    %% Feedback Control for ROBOT 5
    % Compute the controller outputs, i.e., the inputs to the robot
    [r5_v, r5_omega] = r5_controller(r5_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r5_distanceToGoal = norm(r5_robotCurrentPose(1:2) - r5_Goal);
    if (r5_distanceToGoal < goalRadius) || (tFault ~= 0)
        % If the robot reaches within the goal radius, STOP
        r5_cmdvelmsg.Linear.X = 0;
        r5_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r5_cmdvelmsg.Linear.X = r5_v;
        r5_cmdvelmsg.Angular.Z = r5_omega;
    end
    
    % Send the message
    send(r5_cmdvelpub,r5_cmdvelmsg)
    
    % Receive a message from the topic
    r5_odommsg = r5_odomsub.receive;
    
    % Transform quternion to euler
    r5_eularAngles = quat2eul([r5_odommsg.Pose.Pose.Orientation.W,...
        r5_odommsg.Pose.Pose.Orientation.X,...
        r5_odommsg.Pose.Pose.Orientation.Y,...
        r5_odommsg.Pose.Pose.Orientation.Z]);
    r5_yaw = r5_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r5_robotCurrentPose = [r5_odommsg.Pose.Pose.Position.X ...
        r5_odommsg.Pose.Pose.Position.Y ...
        r5_yaw];
    r5_traj(ct,:) = r5_robotCurrentPose;
    

%% Feedback Control for ROBOT 6
    % Compute the controller outputs, i.e., the inputs to the robot
    [r6_v, r6_omega] = r6_controller(r6_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r6_distanceToGoal = norm(r6_robotCurrentPose(1:2) - r6_Goal);
    if (r6_distanceToGoal < goalRadius) || (tFault ~= 0)
        % If the robot reaches within the goal radius, STOP
        r6_cmdvelmsg.Linear.X = 0;
        r6_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r6_cmdvelmsg.Linear.X = r6_v;
        r6_cmdvelmsg.Angular.Z = r6_omega;
    end
    
    % Send the message
    send(r6_cmdvelpub,r6_cmdvelmsg)
    
    % Receive a message from the topic
    r6_odommsg = r6_odomsub.receive;
    
    % Transform quternion to euler
    r6_eularAngles = quat2eul([r6_odommsg.Pose.Pose.Orientation.W,...
        r6_odommsg.Pose.Pose.Orientation.X,...
        r6_odommsg.Pose.Pose.Orientation.Y,...
        r6_odommsg.Pose.Pose.Orientation.Z]);
    r6_yaw = r6_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r6_robotCurrentPose = [r6_odommsg.Pose.Pose.Position.X ...
        r6_odommsg.Pose.Pose.Position.Y ...
        r6_yaw];
    r6_traj(ct,:) = r6_robotCurrentPose;

    
 %% Feedback Control for ROBOT 7
    % Compute the controller outputs, i.e., the inputs to the robot
    [r7_v, r7_omega] = r7_controller(r7_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r7_distanceToGoal = norm(r7_robotCurrentPose(1:2) - r7_Goal);
    if (r7_distanceToGoal < goalRadius) || (tFault ~= 0)
        % If the robot reaches within the goal radius, STOP
        r7_cmdvelmsg.Linear.X = 0;
        r7_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r7_cmdvelmsg.Linear.X = r7_v;
        r7_cmdvelmsg.Angular.Z = r7_omega;
    end
    
    % Send the message
    send(r7_cmdvelpub,r7_cmdvelmsg)
    
    % Receive a message from the topic
    r7_odommsg = r7_odomsub.receive;
    
    % Transform quternion to euler
    r7_eularAngles = quat2eul([r7_odommsg.Pose.Pose.Orientation.W,...
        r7_odommsg.Pose.Pose.Orientation.X,...
        r7_odommsg.Pose.Pose.Orientation.Y,...
        r7_odommsg.Pose.Pose.Orientation.Z]);
    r7_yaw = r7_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r7_robotCurrentPose = [r7_odommsg.Pose.Pose.Position.X ...
        r7_odommsg.Pose.Pose.Position.Y ...
        r7_yaw];
    r7_traj(ct,:) = r7_robotCurrentPose;
    

%% Feedback Control for ROBOT 8
    % Compute the controller outputs, i.e., the inputs to the robot
    [r8_v, r8_omega] = r8_controller(r8_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r8_distanceToGoal = norm(r8_robotCurrentPose(1:2) - r8_Goal);
    if (r8_distanceToGoal < goalRadius) || (tFault ~= 0)
        % If the robot reaches within the goal radius, STOP
        r8_cmdvelmsg.Linear.X = 0;
        r8_cmdvelmsg.Angular.Z = 0;
    else
        % Keep sending a message until it reaches the goal
        r8_cmdvelmsg.Linear.X = r8_v;
        r8_cmdvelmsg.Angular.Z = r8_omega;
    end
    
    % Send the message
    send(r8_cmdvelpub,r8_cmdvelmsg)
    
    % Receive a message from the topic
    r8_odommsg = r8_odomsub.receive;
    
    % Transform quternion to euler
    r8_eularAngles = quat2eul([r8_odommsg.Pose.Pose.Orientation.W,...
        r8_odommsg.Pose.Pose.Orientation.X,...
        r8_odommsg.Pose.Pose.Orientation.Y,...
        r8_odommsg.Pose.Pose.Orientation.Z]);
    r8_yaw = r8_eularAngles(1);

    % Define the new pose by extracting data from the message received
    r8_robotCurrentPose = [r8_odommsg.Pose.Pose.Position.X ...
        r8_odommsg.Pose.Pose.Position.Y ...
        r8_yaw];
    r8_traj(ct,:) = r8_robotCurrentPose;
    
    %% ATTACK INJECTION TO ROS
    if ct >= att_t   % Fault injection time (sec)
        fprintf('Now Under Attack!')
        r0_robotCurrentPose(att_state) = randi(7);
        r2_robotCurrentPose(att_state) = randi(7);
        r8_robotCurrentPose(att_state) = randi(7);
    end
    
     waitfor(controlRate);
end
%toc

%% Plots
%run plot_ani_traj.m;
%run plot_ani_stat.m;