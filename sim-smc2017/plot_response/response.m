clear; close all; clc;

M = 1;  % units of kg
K = 10; % units of N/m
B = 2;  % units of N-s/m
num = 10;
den = [M B K];
sys = tf(num,den,'InputDelay',4);

figure; 
step(sys,'b')
