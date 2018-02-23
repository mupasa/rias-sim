%Initializing the agents to random positions with barrier certificates 
%and data plotting.  This script shows how to initialize robots to a
%particular pose
%Paul Glotfelter 
%3/24/2016

% Modified by Sangjun Lee
% lee1424@purdue.edu
% 2/16/2018

clear; close all; clc;

% Import Data
load rundata.mat


%% Figure
lineW = 1.2;
axisS = 15;
labelS = 15;

% Trajectory of all agents
figure; 
plot(data.pose(1:3:end,:), data.pose(2:3:end,:))

grid on; axis([-1.5 1.5 -1.5 1.5]);
legend({'R1','R2','R3','R4','R5','R6','R7','R8'},'Location','Northwest','FontSize',labelS);
xlabel('x [meter]','FontSize',labelS); ylabel('y [meter]','FontSize',labelS)

% Trajectory of compromised agent
figure; 
plot(data.pose(data.att_state:3:end,data.att_agent), ...
    data.pose(data.att_state+1:3:end,data.att_agent))
grid on; axis([-1.5 1.5 -1.5 1.5]);
