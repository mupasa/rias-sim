%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research
% MATLAB-GAZEBO Multi-Robot Simulation
% 
% MATLAB Configuration
%  - Run the detection algorithm
%
% ROS Nodes Configuration
%  - Run GAZEBO simulator
%
% Useful Topics
%  - Control input: '/rX/husky_velocity_controller/cmd_vel'
%  - Ground truth (Abs Pose) : '/rX/base_pose_ground_truth'
%  - Odometry reading (Local Pose): '/rX/husky_velocity_controller/odom'
%
% Change intiail locations of robots
%  - ~/catkin_ws/src/nre_simultihusky/launch/multi_vehicle.launch
%
% Created by Sangjun Lee
% 9/7/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc; 
%tic;


%% Running ROS core and Gazebo
% export ROS_MASTER_URI=http://128.46.80.246:11311
% export ROS_IP=128.46.80.246

% roscore
% roslaunch nre_simmultihusky multi_vehicle.launch

% Echo some interesting topics
% rostopic echo /rX/husky_velocity_controller/cmd_vel
% rostopic echo /rX/husky_velocity_controller/odom
% rostopic echo /r1/base_pose_ground_truth


%% Connect to the ROS master
% setenv('ROS_MASTER_URI','http://128.46.80.235:11311') % ROS IP
% setenv('ROS_IP','128.46.80.246') % MATLAB IP
% rosinit


%% Shut down the ROS network
% rosshutdown


%% Simulation Design Variables

% State vector.
%disp('State Vector: x = [x y theta vx vy thetadot ax ay bthetadot bax bay]')

% Set the sampling time (sec)
Ts = 0.1;  % 10 Hz

% Attack Profile
att_t = 30;            % Injection time (sec)
att_mag = deg2rad(60); % Amount (rad)
att_robot = 3;         % Target robot
att_state = 3;         % Target state

fprintf('Attack Profile: %d sec, %d rad, Robot No. %d, State No. %d\n',...
    att_t,att_mag,att_robot, att_state)

% Define the controller rate
controlRate = robotics.Rate(1/Ts); % Hz

% Set the goal location of the robots
rdv = [30 30]; % Rendezvous point
r1_Goal = rdv; % r1 initial location = [0 0]
r2_Goal = rdv; % r2 initial location = [60 10]
r3_Goal = rdv; % r3 initial location = [20 70]

fprintf('Rendezvous at (%d, %d)\n',rdv(1),rdv(2))

% Set the contoller specifications
DesiredLinearVelocity = 1; % 1m/sec = 3.6km/h
MaxAngularVelocity = 0.5; % rad/sec
LookaheadDistance = 5; % m
goalRadius = 2; % m


%% Define ROS Topic Publisher and Subscribers

% Publish topics as control inputs
r1_cmdvelpub = rospublisher('/r1/husky_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r1_cmdvelmsg = rosmessage(r1_cmdvelpub);
r2_cmdvelpub = rospublisher('/r2/husky_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r2_cmdvelmsg = rosmessage(r2_cmdvelpub);
r3_cmdvelpub = rospublisher('/r3/husky_velocity_controller/cmd_vel', 'geometry_msgs/Twist');
r3_cmdvelmsg = rosmessage(r3_cmdvelpub);

% Subscribe topics as feedback
r1_odomsub = rossubscriber('/r1/base_pose_ground_truth');
r2_odomsub = rossubscriber('/r2/base_pose_ground_truth');
r3_odomsub = rossubscriber('/r3/base_pose_ground_truth');


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
r3_initmsg = r3_odomsub.receive;
r3_robotCurrentLocation = [r3_initmsg.Pose.Pose.Position.X,...
    r3_initmsg.Pose.Pose.Position.Y];

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


%% Detector Design

% Initial state.
x0 = [0 0 deg2rad(0) 0 0 0 0 0 0 0 0]';

% State error covariance.
P = diag([2.5^2 2.5^2 deg2rad(0.1) 1e-02 1e-02 deg2rad(0.1)...
    1e-04 1e-04 deg2rad(0.1) 1e-01 1e-01]);

% Process noise covariance.
Q = diag([1e-04 1e-04 deg2rad(0.01) 1e-06 1e-06 deg2rad(0.01)...
    1e-09 1e-09 deg2rad(0.01) 1e-05 1e-05]);

% Measurement noise covariance.
R = diag([2.5^2 2.5^2 deg2rad(0.1) deg2rad(0.1) 1e-01 1e-01]);

% Create an extended Kalman Filter to estimate the states of the model.
ekf = extendedKalmanFilter(...
    @stateUpdate_MotorModel, ...
    @measurement_MotorModel, ...
    x0,...
    'StateCovariance',            P, ...
    'ProcessNoise',               Q, ...
    'MeasurementNoise',           R, ...
    'StateTransitionJacobianFcn', @stateJacobian_MotorModel, ...
    'MeasurementJacobianFcn',     @measurementJacobian_MotorModel);


%% Simulation

tEnd = 60;                     % Final time (sec)
t  = 0:Ts:tEnd;                % Simulation time with Ts sampling period
u = 10*sin(t*(2*pi)/60);       % INPUT: Sine wave, amplitude 10, period (2*pi)/60
nt = numel(t);                 % Number of time points
nx = size(x0,1);               % Number of states
ySig = zeros([6, nt]);         % Measured motor outputs
xSigTrue = zeros([nx, nt]);    % Unmeasured motor states
xSigEst = zeros([nx, nt]);     % Estimated motor states
xstd = zeros([nx nx nt]);      % Standard deviation of the estimated states 
ySigEst = zeros([6, nt]);      % Estimated model outputs
fMean = zeros(1,nt);           % Mean estimated friction
fSTD = zeros(1,nt);            % Standard deviation of estimated friction 
fKur = zeros(2,nt);            % Kurtosis of estimated friction
fChanged = false(1,nt);        % Flag indicating friction change detection, false=0 
estFriction = zeros(1,nt);     % Test statistics
tFault = zeros(1);             % Alarm time (sec)
i = 1;                         % Number of detected fault

rng('default');
Qv = chol(Q);   % Standard deviation for process noise
Qv(end) = 1e-2; % Smaller friction noise
Rv = chol(R);   % Standard deviation for measurement noise

for ct = 1:numel(t) % Main Loop
%% Feedback Control for ROBOT 1
    % Compute the controller outputs, i.e., the inputs to the robot
    [r1_v, r1_omega] = r1_controller(r1_robotCurrentPose);
    
    % Send a message to drive the robot until it reaches the goal
    r1_distanceToGoal = norm(r1_robotCurrentPose(1:2) - r1_Goal);
    if r1_distanceToGoal < goalRadius
        % If the robot reaches within the goal radius, STOP
        r1_cmdvelmsg.Linear.X = 0;
        r1_cmdvelmsg.Angular.Z = 0;
        send(r1_cmdvelpub,r1_cmdvelmsg)
    else
        % Keep sending a message until it reaches the goal
        r1_cmdvelmsg.Linear.X = r1_v;
        r1_cmdvelmsg.Angular.Z = r1_omega;
        send(r1_cmdvelpub,r1_cmdvelmsg)
    end
    
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
    if r2_distanceToGoal < goalRadius
        % If the robot reaches within the goal radius, STOP
        r2_cmdvelmsg.Linear.X = 0;
        r2_cmdvelmsg.Angular.Z = 0;
        send(r2_cmdvelpub,r2_cmdvelmsg)
    else
        % Keep sending a message until it reaches the goal
        r2_cmdvelmsg.Linear.X = r2_v;
        r2_cmdvelmsg.Angular.Z = r2_omega;
        send(r2_cmdvelpub,r2_cmdvelmsg)
    end
    
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
    if r3_distanceToGoal < goalRadius
        % If the robot reaches within the goal radius, STOP
        r3_cmdvelmsg.Linear.X = 0;
        r3_cmdvelmsg.Angular.Z = 0;
        send(r3_cmdvelpub,r3_cmdvelmsg)
    else
        % Keep sending a message until it reaches the goal
        r3_cmdvelmsg.Linear.X = r3_v;
        r3_cmdvelmsg.Angular.Z = r3_omega;
        send(r3_cmdvelpub,r3_cmdvelmsg)
    end
    
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
    
    
%% ATTACK INJECTION TO ROS
    if t(ct) >= att_t   % Fault injection time (sec)
        if att_robot == 1
            r1_robotCurrentPose(att_state) = att_mag;
        elseif att_robot == 2 
            r2_robotCurrentPose(att_state) = att_mag;
        elseif att_robot == 3
            r3_robotCurrentPose(att_state) = att_mag;
        end
    end
    
    waitfor(controlRate);

%% Estimation
   % Model output update   
   y = measurement_MotorModel(x0,u(ct),Ts);
   y = y + Rv*randn(6,1);   % Add measurement noise
   ySig(:,ct) = y;
   
   % Model state update 
   xSigTrue(:,ct) = x0;
   x1 = stateUpdate_MotorModel(x0,u(ct),Ts);
   
   % ATTACK INJECTION TO MATLAB
   if t(ct) == att_t   % Fault injection time (sec)
      x1(att_state) = att_mag;  % Step change (Amount of fault)
      
   end
   
   x1n = x1 + Qv*randn(nx,1);  % Add process noise
   x1n(att_state) = max(x1n(att_state),0.1); % Lower limit on friction
   x0 = x1n; % Store state for next simulation iteration       

   % State estimation using the Extended Kalman Filter
   x_corr = correct(ekf,y,u(ct),Ts); % Correct the state estimate based on current measurement.
   xSigEst(:,ct) = x_corr;
   xstd(:,:,ct) = chol(ekf.StateCovariance);
   predict(ekf,u(ct),Ts);            % Predict next state given the current state and input.
        
%% Detection Algorithm

   if t(ct) < 10 % Ts = 0.01 seconds
       % Compute mean and standard deviation of estimated friction.
       idx = max(1,ct-1000):max(1,ct-1); % 10 second moving window
       fMean(ct) = mean( xSigEst(att_state, idx) );
       fSTD(ct)  = std( xSigEst(att_state, idx) );
   else
       % Store the computed mean and standard deviation without
       % recomputing.
       fMean(ct) = fMean(ct-1);
       fSTD(ct)  = fSTD(ct-1);
       % Use the expected friction mean and standard deviation to detect
       % friction changes.
       estFriction(ct) = mean(xSigEst(att_state,max(1,ct-10):ct));
       fChanged(ct) = (estFriction(ct) > fMean(ct)+3*fSTD(ct)) || (estFriction(ct) < fMean(ct)-3*fSTD(ct));
   end
   
   if fChanged(ct) && ~fChanged(ct-1) 
       % Detect a rising edge in the friction change signal |fChanged|.
       fprintf('An attack has been detected at %f\n',t(ct));
       tFault(i) = t(ct);
       i = i + 1;
   end
end
%toc

%% Plots
LineW = 1.5;
FontS_axis = 15;
FontS_label = 18;

figure; % DOA state estimation error
plot(t,xSigTrue(att_state,:)-xSigEst(att_state,:),'b','LineWidth',LineW); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('angle (rad)','Fontsize',FontS_label);

figure; % DOA measurement error
plot(t,ySig(att_state,:)-ySigEst(att_state,:),'b','LineWidth',LineW); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('angle (rad)','Fontsize',FontS_label);

figure; % Residual evolution
plot(t,xSigEst(att_state,:),'b',...
    [t nan t],[fMean+3*fSTD,nan,fMean-3*fSTD],'r--','LineWidth',LineW); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('S(k)','Fontsize',FontS_label);
legend({'Residual','Threshold'},'Location','Northeast','FontSize',FontS_label)

figure; % State history comparison
subplot(3,1,1); plot(t,r1_traj(:,att_state),'b','LineWidth',LineW);
title('Heading Angles (rad)')
ylabel('\theta_{R1}'); grid on; ax = gca; ax.FontSize = FontS_axis;
subplot(3,1,2); plot(t,r2_traj(:,att_state),'b','LineWidth',LineW); 
ylabel('\theta_{R2}'); grid on; ax = gca; ax.FontSize = FontS_axis;
subplot(3,1,3); plot(t,r3_traj(:,att_state),'b','LineWidth',LineW); 
ylabel('\theta_{R3}'); grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)')

figure; % Trajectory
plot(r1_traj(:,1), r1_traj(:,2),'r',...
    r2_traj(:,1), r2_traj(:,2),'g',...
    r3_traj(:,1), r3_traj(:,2),'b',...
    r3_controller.Waypoints(2,1),r3_controller.Waypoints(2,2),'k*',...
    r1_traj(end,1),r1_traj(end,2),'r>',...
    r2_traj(end,1),r2_traj(end,2),'g<',...
    r3_traj(end,1),r3_traj(end,2),'b<','LineWidth',LineW)
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('East (m)','Fontsize',FontS_label); 
ylabel('North (m)','Fontsize',FontS_label);
legend({'R1 actual trajectory','R2 actual trajectory','R3 actual trajectory',...
    'Rendezvous point'},'Location','Northeast','FontSize',FontS_label)
