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
% /catcar_ws/src/catvehicle-2.0.2/worlds/fdirHigh.world
% /catcar_ws/src/catvehicle-2.0.2/launch/catvehicle_fdirHigh.launch
%
% Created by Sangjun Lee
% 5/8/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc; 
%tic;


%% (ROS)
% export ROS_IP=128.46.80.246
% export ROS_MASTER_URI=http://128.46.80.246:11311

% roscore
% roslaunch catvehicle catvehicle_fdirHigh.launch
% roslaunch gzclient

% Echo some important topics
% rostopic echo /catvehicle/odom
% rostopic echo /catvehicle/waypoint
% rostopic echo /catvehicle/cmd_vel


%% Connect to the ROS master
% setenv('ROS_MASTER_URI','http://128.46.80.242:11311') % ROS IP
% setenv('ROS_IP','128.46.80.240') % MATLAB IP
% rosinit


%% Detector Design

% Sample time (sec)
Ts = 0.1;

% Attack injection time (sec)
t_inject = 40;

% State vector.
%disp('State Vector: x = [x y theta vx vy thetadot ax ay bthetadot bax bay]')

% Initial state.
x0 = [0 0 deg2rad(90) 0 0 0 0 0 0 0 0]';

% State error covariance.
P = diag([2.5^2 2.5^2 deg2rad(1e-02) 1e-02 1e-02 deg2rad(1e-02)...
    1e-04 1e-04 deg2rad(1e-02) 1e-01 1e-01]);

% Process noise covariance.
Q = diag([1e-04 1e-04 deg2rad(1e-06) 1e-06 1e-06 deg2rad(1e-06)...
    1e-09 1e-09 deg2rad(1e-05) 1e-05 1e-05]);

% Measurement noise covariance.
R = diag([2.5^2 2.5^2 deg2rad(1e-02) deg2rad(1e-02) 1e-01 1e-01]);

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


%% Controller Design

% Define a set of waypoints for the desired path for the robot
% path = [0.00    0.00
%     12.00   10.00
%     20.00 15.00];
% path = [0.00    0.00
%     30.00   0.00
%     30.00   30.00
%     0.00    30.00];

path = [0 0 ; 700 0]

% Set the current location and the goal location of the robot as defined by the path
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

% Set an initial robot orientation 
initialOrientation = 0;

% Define the current pose for the robot [x y theta]
robotCurrentPose = [robotCurrentLocation initialOrientation];

% Define the path following controller
controller = robotics.PurePursuit;

% Use the path defined above to set the desired waypoints for the controller
controller.Waypoints = path;

% Set the path following controller parameters.
controller.DesiredLinearVelocity = 1; % 1m/sec = 3.6km/h
controller.MaxAngularVelocity = 1; % rad/sec
controller.LookaheadDistance = 3; % m

% Define the controller rate
controlRate = robotics.Rate(1/Ts); % Hz

% Publish the topics for ROS
cmdvelpub = rospublisher('/catvehicle/cmd_vel', 'geometry_msgs/Twist');
cmdvelmsg = rosmessage(cmdvelpub);

% Subscribe to the topic for ROS
odomsub = rossubscriber('/catvehicle/odom');


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
tAttack = zeros(1);             % Alarm time (sec)
i = 1;                         % Number of detected fault
j = 1;                         % 

rng('default');
Qv = chol(Q);   % Standard deviation for process noise
Qv(end) = 1e-2; % Smaller friction noise
Rv = chol(R);   % Standard deviation for measurement noise

for ct = 1:numel(t)
%% Path Tracking
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
    robotTraj(ct,:) = robotCurrentPose;
    
    % Induce change in position
    if t(ct) >= t_inject   % Fault injection time, GPS is fixed after 35s
        robotCurrentPose(2) = 10;  % Step change (Amount of fault)
    end
    
    % Increase the speed
    if controller.DesiredLinearVelocity < 17 % 17m/s = 60km/h
        controller.DesiredLinearVelocity = controller.DesiredLinearVelocity+1;
    end

%% Estimation
   % Model output update   
   y = measurement_MotorModel(x0,u(ct),Ts);
   y = y + Rv*randn(6,1);   % Add measurement noise
   ySig(:,ct) = y;
   
   % Model state update 
   xSigTrue(:,ct) = x0;
   x1 = stateUpdate_MotorModel(x0,u(ct),Ts);
   
   % Induce change in position
   if t(ct) == t_inject   % Fault injection time, GPS is fixed after 35s
      x1(2) = 10;  % Step change (Amount of fault)
   end
   
   x1n = x1 + Qv*randn(nx,1);  % Add process noise
   x1n(2) = max(x1n(2),0.1); % Lower limit on friction
   x0 = x1n; % Store state for next simulation iteration       

   % State estimation using the Extended Kalman Filter
   x_corr = correct(ekf,y,u(ct),Ts); % Correct the state estimate based on current measurement.
   xSigEst(:,ct) = x_corr;
   xstd(:,:,ct) = chol(ekf.StateCovariance);
   predict(ekf,u(ct),Ts);            % Predict next state given the current state and input.
        
%% Detection Algorithm

   if t(ct) < 10 % Ts = 0.1 seconds
       % Compute mean and standard deviation of estimated friction.
       idx = max(1,ct-1000):max(1,ct-1); % 10 second moving window
       fMean(ct) = mean( xSigEst(2, idx) );
       fSTD(ct)  = std( xSigEst(2, idx) );
   else
       % Store the computed mean and standard deviation without
       % recomputing.
       fMean(ct) = fMean(ct-1);
       fSTD(ct)  = fSTD(ct-1);
       % Use the expected friction mean and standard deviation to detect
       % friction changes.
       estFriction(ct) = mean(xSigEst(2,max(1,ct-10):ct));
       fChanged(ct) = (estFriction(ct) > fMean(ct)+3*fSTD(ct)) || (estFriction(ct) < fMean(ct)-3*fSTD(ct));
   end
   
   if fChanged(ct) && ~fChanged(ct-1) 
       % Detect a rising edge in the friction change signal |fChanged|.
       fprintf('An attack has been injected at %f\n',t_inject);
       fprintf('An attack has been detected at %f\n',t(ct));
       tAttack(i) = t(ct);
       i = i + 1;
       break
   end
   
   waitfor(controlRate);
end
%toc

%% Plots
LineW = 1.5;
FontS_axis = 15;
FontS_label = 18;

figure; % North state estimation error
plot(t,xSigTrue(2,:)-xSigEst(2,:),'b','LineWidth',LineW); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('North (m)','Fontsize',FontS_label);

figure; % North measurement error
plot(t,ySig(2,:)-ySigEst(2,:),'b'); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('North (m)','Fontsize',FontS_label);

figure; % Trajectory
plot(path(:,1), path(:,2),'k--d',...
    robotTraj(:,1), robotTraj(:,2),'b','LineWidth',LineW)
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('East (m)','Fontsize',FontS_label); ylabel('North (m)','Fontsize',FontS_label);
legend({'Desired waypoints','Actual trajectory'},'Location','Northwest','FontSize',FontS_label)

figure; % Residual evolution
plot(t,xSigEst(2,:),'b',...
    [t nan t],[fMean+3*fSTD,nan,fMean-3*fSTD],'r--','LineWidth',LineW); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('S(k)','Fontsize',FontS_label);
legend({'Residual','Threshold'},'Location','Northwest','FontSize',FontS_label)


%% Shut down the ROS network
% rosshutdown
