%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research: Fault Detection Using an Extended Kalman Filter
% Created by Sangjun Lee
% 3/31/2017
% ros_main_rum.m
% This script is to run a ros node as the master and send a message 
% to subscribers if there is a fault.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;

%% (ROS)
% export ROS_IP=128.46.80.246
% export ROS_MASTER_URI=http://128.46.80.246:11311

% Run roscore
% roscore

%% Connect to the ROS master (roscore)
setenv('ROS_MASTER_URI','http://128.46.80.246:11311') % ROS IP
setenv('ROS_IP','128.46.80.244') % MATLAB IP
rosinit

% Create a message
attackpub = rospublisher('/detector', 'std_msgs/Int64');
attackmsg = rosmessage(attackpub);

% (ROS) rostopic echo /detector


%% Vehicle parameters
Ts = 0.01;  % Sample time (sec)

%% State
% State vector.
disp('State Vector: x = [x y theta vx vy thetadot ax ay bthetadot bax bay]')
% Initial state.
x0 = [0 0 deg2rad(90) 0 0 0 0 0 0 0 0]';

%% Covariances
% State error covariance.
P = diag([2.5^2 2.5^2 deg2rad(1e-02) 1e-02 1e-02 deg2rad(1e-02)...
    1e-04 1e-04 deg2rad(1e-02) 1e-01 1e-01]);

% Process noise covariance.
Q = diag([1e-04 1e-04 deg2rad(1e-06) 1e-06 1e-06 deg2rad(1e-06)...
    1e-09 1e-09 deg2rad(1e-05) 1e-05 1e-05]);

% Measurement noise covariance.
R = diag([2.5^2 2.5^2 deg2rad(1e-02) deg2rad(1e-02) 1e-01 1e-01]);

%% Creating an Extended Kalman Filter
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

for ct = 1:numel(t)
    
   % Model output update   
   y = measurement_MotorModel(x0,u(ct),Ts);
   y = y + Rv*randn(6,1);   % Add measurement noise
   ySig(:,ct) = y;
   
   % Model state update 
   xSigTrue(:,ct) = x0;
   x1 = stateUpdate_MotorModel(x0,u(ct),Ts);
   
   % Induce change in friction
   if t(ct) == 40   % Fault injection time, GPS is fixed after 35s
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

   if t(ct) < 10 % Ts = 0.01 seconds
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
       fprintf('Significant Fault at %f\n',t(ct));
       tFault(i) = t(ct);
       i = i + 1;
       attackmsg.Data = 1; % Send an alert to listener
       send(attackpub,attackmsg)
%        alertmsg.Data = num2str(t(ct));
%        send(alerttime,alertmsg)
%    else
%        attackmsg.Data = false; % Send a message to listener

   end
   %pause(Ts)
end
