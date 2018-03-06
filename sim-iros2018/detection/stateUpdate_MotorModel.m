function x1 = stateUpdate_MotorModel(x0,varargin)
%STATEUPDATE_MOTORMODEL
%
% State update equations for a motor with friction as a state
%
%  x1 = stateUpdate_MotorModel(x0,u,J,Ts)
%
%  Inputs:
%    x0 - initial state with elements [angular velocity; friction] 
%    u  - motor torque input
%    J  - motor inertia
%    Ts - sampling time
%
%  Outputs:
%    x1 - updated states
%

%  Copyright 2016 The MathWorks, Inc.

% Extract data from inputs
u  = varargin{1};   % Input
Ts = varargin{2};   % Sample time

%x0(1) = u;
% State update equation
x1 = [
    x0(1) + Ts*x0(4)*cos(x0(3)) - Ts*x0(5)*sin(x0(3))
    x0(2) + Ts*x0(4)*sin(x0(3)) + Ts*x0(5)*cos(x0(3))
    x0(3) + Ts*x0(6)
    x0(4) + Ts*x0(7)
    x0(5) + Ts*x0(8)
    x0(6)
    x0(7)
    x0(8)
    x0(9)
    x0(10)
    x0(11)];
    
end