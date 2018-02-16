%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FDIR Research
% Generate Jacobian Matrix
% Created by Sangjun Lee
% 3/31/2017
% main_rum.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all; close all; clc;

% Define State vector as symbol
syms x y theta vx vy thetadot ax ay bthetadot bax bay T

% State vector
state = [x y theta vx vy thetadot ax ay bthetadot bax bay];

% State Model
f = [
    x + T*vx*cos(theta) - T*vy*sin(theta)
    y + T*vx*sin(theta) + T*vy*cos(theta)
    theta + T*thetadot
    vx + T*ax
    vy + T*ay
    thetadot
    ax
    ay
    bthetadot
    bax
    bay];

% Measurement Model
h =[x y theta thetadot+bthetadot ax+bax ay+bay]'; 

% Generate the Jacobian Matrices
F = jacobian(f, state)
H = jacobian(h, state)