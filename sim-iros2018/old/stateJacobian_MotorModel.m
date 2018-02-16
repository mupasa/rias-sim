function Jac_F = stateJacobian_MotorModel(x,varargin)
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
%J  = varargin{2};
u  = varargin{1};   % Input
Ts = varargin{2};   % Smapling Time

% Jacobian
% Jac_F = [...
%     1-Ts/J*x(2) -Ts/J*x(1); ...
%     0 1];

% SJ
Jac_F = [ 
    1, 0, - Ts*x(5)*cos(x(3)) - Ts*x(4)*sin(x(3)), Ts*cos(x(3)), -Ts*sin(x(3)),  0,   0,  0, 0, 0, 0
    0, 1,   Ts*x(4)*cos(x(3)) - Ts*x(5)*sin(x(3)), Ts*sin(x(3)),  Ts*cos(x(3)),  0,   0,  0, 0, 0, 0
    0, 0,                                       1,            0,             0, Ts,   0,  0, 0, 0, 0
    0, 0,                                       0,            1,             0,  0,  Ts,  0, 0, 0, 0
    0, 0,                                       0,            0,             1,  0,   0, Ts, 0, 0, 0
    0, 0,                                       0,            0,             0,  1,   0,  0, 0, 0, 0
    0, 0,                                       0,            0,             0,  0,   1,  0, 0, 0, 0
    0, 0,                                       0,            0,             0,  0,   0,  1, 0, 0, 0
    0, 0,                                       0,            0,             0,  0,   0,  0, 1, 0, 0
    0, 0,                                       0,            0,             0,  0,   0,  0, 0, 1, 0
    0, 0,                                       0,            0,             0,  0,   0,  0, 0, 0, 1];

end