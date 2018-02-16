function Jac = stateJacobian_MotorModel(x,varargin)
%STATEJACOBIAN_MOTORMODEL
%
% Jacobian of motor model state equations. See stateUpdate_MotorModel for
% the model equations.
%
%  Jac = stateJacobian_MotorModel(x,u,J,Ts)
%
%  Inputs:
%    x  - state with elements [angular velocity; friction] 
%    u  - motor torque input
%    J  - motor inertia
%    Ts - sampling time
%
%  Outputs:
%    Jac - state Jacobian computed at x
%

%  Copyright 2016 The MathWorks, Inc.

% Model properties
J  = varargin{2};
Ts = varargin{3};

% Jacobian
Jac = [...
    1-Ts/J*x(2) -Ts/J*x(1); ...
    0 1];
end