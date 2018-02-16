%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research: Fault Detection Using an Extended Kalman Filter
% Created by Sangjun Lee
% 4/10/2017
% talk2ROS.m
% This script is to run a ros node as the master and send a message 
% to subscribers if there is a fault.
% (external) represents a task that should be done on the external device
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;

%% Initialization
% Create the ROS master
rosinit

% Connect to the ROS master
%rosinit('128.46.80.245',11311)

% (external) Connect the master and see if it works
% export ROS_MASTER_URI=http://bo-80-16.dhcp.ecn.purdue.edu:11311
% rosnode list
% You will see something like /matlab_global_node_19730

% Create a message
attackpub = rospublisher('/detector', 'std_msgs/Int64');
attackmsg = rosmessage(attackpub);

% Create a message
% alerttime = rospublisher('/alert', 'std_msgs/String');
% alertmsg = rosmessage(alerttime);

% (external) Run rostopic echo to see if msg has been arrived
% rostopic echo /detector

%% Main run
% Run ros_main_run.m
run ros_main_run.m

%% Shut down the ROS network
% rosshutdown