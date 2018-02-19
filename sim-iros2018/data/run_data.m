% Robotarium Test Data Plot
% Sangjun Lee
% 2/19/2018
clear; close all; clc;

% Import the data
load('2_17_2018_data.mat');

% Generate figure 
[lim_row, lim_col] = size(robotarium_data);
 
figure; hold on;
j = 1;
while j < lim_row
    plot(robotarium_data(j,:),robotarium_data(j+1,:));
    j = j + 5;
end

grid on; axis([-1.5 1.5 -1.5 1.5]);
legend('1','2','3','4','5','6','7','8');
xlabel('x [meter]'); ylabel('y [meter]')