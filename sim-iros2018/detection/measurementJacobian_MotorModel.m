function Jac_H = measurementJacobian_MotorModel(x,varargin)
%MEASUREMENT_MOTORMODEL
%
% Jacobian of motor model measurement equations. See measurement_MotorModel for
% the model equations.
%
%  Jac = measurementJacobian_MotorModel(x,u,J,Ts)
%
%  Inputs:
%    x  - state with elements [angular velocity; friction] 
%    u  - motor torque input
%    J  - motor inertia
%    Ts - sampling time
%
%  Outputs:
%    Jac - measurement Jacobian computed at x
%

%  Copyright 2016 The MathWorks, Inc.

% System parameters
%J  = varargin{2};   % System innertia

% Jacobian
% Jac_H = [ ...
%     1 0;
%     -x(2)/J -x(1)/J];

% SJ
Jac_H =[ 
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0
    0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0
    0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1];
end
