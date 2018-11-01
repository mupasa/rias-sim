function [t,xTrue] = generateDataTrueStates(Ts)

%   Copyright 2016 The MathWorks, Inc.

% An object is moving in a square pattern in a 2D plane, each side is 1000m. 
% Velocity when traveling side 1: 50m/s
%                         side 2: 40m/s
%                         side 3: 25m/s
%                         side 4: 10m/s
%
% Assume O is the origin (0,0), and the objects starts from (100,100)
%
%         2
%    ----->-----
%   |           |
%   |          \|/ 3
%  /|\ 1        |
%   |           |
%   O-----<-----
%         4

% Create true position
xEast = zeros(185/Ts,1); % 185s is the total travel time
xNorth = zeros(185/Ts,1);
vEast = zeros(185/Ts,1);
vNorth = zeros(185/Ts,1);
% position in side 1
t = (0:Ts:20-Ts).';
xNorth(1:20/Ts) = 50*t;
vNorth(1:20/Ts) = 50;
% position in side 2
t = (0:Ts:25-Ts).';
xEast(20/Ts+1:45/Ts) = 40*t;
xNorth(20/Ts+1:45/Ts) = 1000;
vEast(20/Ts+1:45/Ts) = 40;
% position in side 3
t = (0:Ts:40-Ts).';
xEast(45/Ts+1:85/Ts) = 1000;
xNorth(45/Ts+1:85/Ts) = 1000-25*t;
vNorth(45/Ts+1:85/Ts) = -25;
% position in side 4
t = (0:Ts:100-Ts).';
xEast(85/Ts+1:185/Ts) = 1000-10*t;
xNorth(85/Ts+1:185/Ts) = 0;
vEast(85/Ts+1:185/Ts) = -10;
% Put things together, and add the initial position offset
xTrue = [xEast+100 xNorth+100 vEast vNorth];
clear xEast xNorth vEast vNorth;
% Create time vector
t = (0:Ts:185-Ts).';
end