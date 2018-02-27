%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research
% Fault Detection Using an Extended Kalman Filter
% Created by Sangjun Lee
% 3/31/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;


%% Vehicle parameters
Ts = 0.1;  % Sample time (sec)

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
estEst = zeros(1,nt);     % Test statistics
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
       estEst(ct) = mean(xSigEst(2,max(1,ct-10):ct));
       fChanged(ct) = (estEst(ct) > fMean(ct)+3*fSTD(ct)) || (estEst(ct) < fMean(ct)-3*fSTD(ct));
   end
   if fChanged(ct) && ~fChanged(ct-1) 
       % Detect a rising edge in the friction change signal |fChanged|.
       fprintf('Significant Fault at %f\n',t(ct));
       tFault(i) = t(ct);
       i = i + 1;
   end
end

%% Extended Kalman Filter Performance Evaluation

% Input
figure, 
plot(t,u);
title('Vehicle Input - East')

% Seletive State
figure, 
subplot(311),plot(t,xSigTrue(1,:), t,xSigEst(1,:), ...
    [t nan t],[xSigEst(1,:)+3*squeeze(xstd(1,1,:))', nan, xSigEst(1,:)-3*squeeze(xstd(1,1,:))']) 
legend('True value','Estimated value','Confidence interval')
title('Vehicle State - East')

subplot(312),plot(t,xSigTrue(2,:), t,xSigEst(2,:),  ...
    [t nan t],[xSigEst(2,:)+3*squeeze(xstd(2,2,:))' nan xSigEst(2,:)-3*squeeze(xstd(2,2,:))'])
title('Vehicle State - North');

subplot(313),plot(t,xSigTrue(3,:), t,xSigEst(3,:),  ...
    [t nan t],[xSigEst(3,:)+3*squeeze(xstd(3,3,:))' nan xSigEst(3,:)-3*squeeze(xstd(3,3,:))'])
title('Vehicle State - Heading');

% % Full State
% figure, 
% for j = 3:nx
%     subplot(3,3,j-2),plot(t,xSigTrue(j,:), t,xSigEst(j,:))
%     title(['State ' num2str(j)]);
% end
% legend('True value','Estimated value')

% State Estimation Error
figure, 
subplot(311),plot(t,xSigTrue(1,:)-xSigEst(1,:))
title('East State Error')

subplot(312),plot(t,xSigTrue(2,:)-xSigEst(2,:))
title('North State Error')

subplot(313),plot(t,xSigTrue(3,:)-xSigEst(3,:))
title('Heading State Error')

% Fault Detecction
figure
plot(t,xSigEst(2,:),[t nan t],[fMean+3*fSTD,nan,fMean-3*fSTD])
title('Fault Detection')
legend('Estimated','No-Fault Bounds')
xlabel('time (sec)')
grid on


% Seletive Measurement
figure, 
subplot(311),plot(t,ySig(1,:), t,ySigEst(1,:))
legend('True value','Estimated value')
title('East Measurement')

subplot(312),plot(t,ySig(2,:), t,ySigEst(2,:))
title('North Measurement');

subplot(313),plot(t,ySig(3,:), t,ySigEst(3,:))
title('Heading Measurement');

% Measurement Estimation Error
figure
subplot(311), plot(t,ySig(1,:)-ySigEst(1,:))
title('East Measurement Error')
subplot(312),plot(t,ySig(2,:)-ySigEst(2,:))
title('North Measurement Errror')
subplot(313),plot(t,ySig(3,:)-ySigEst(3,:))
title('Heading Measurement Error')

% % Decision Function
% figure
% plot(t,estEst,...
%     t,ones(ct,1) .* fMean(tFault*100)+3*fSTD(tFault*100),'r',...
%     t,ones(ct,1) .* fMean(tFault*100)-3*fSTD(tFault*100),'r')
% title('Evolution of Decision Function')
% legend('Decision Function','Threshold')
% xlabel('time (sec)')
% axis([0 tEnd -10 20])
% grid on


%% Plots for Paper
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

figure; % Residual evolution
plot(t,xSigEst(2,:),'b',...
    [t nan t],[fMean+3*fSTD,nan,fMean-3*fSTD],'r--','LineWidth',LineW); 
grid on; ax = gca; ax.FontSize = FontS_axis;
xlabel('time (s)','Fontsize',FontS_label); ylabel('S(k)','Fontsize',FontS_label);
legend({'Residual','Threshold'},'Location','Northwest','FontSize',FontS_label)
