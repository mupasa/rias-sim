% Robotarium Test Data Plot
% Sangjun Lee
% 2/19/2018
clear; close all; clc;

%% Import Data
load 2_17_2018_data.mat; % Will generate robotarium_data


%% Run Detection System
[lim_row, lim_col] = size(robotarium_data);
t_s = 50;
t = linspace(0,lim_col/t_s,lim_col);

goal = [1 1 1];
i_atk = 4;


%% Generate Figure
lineW = 1.2;
axisS = 15;
labelS = 15;

% Trajectory of all the agents
figure; 
hold on;
j = 1;
while j < lim_row
    plot(robotarium_data(j,:),robotarium_data(j+1,:),'LineWidth',lineW);
    j = j + 5;
end
hold off;

grid on; axis([-1.5 1.5 -1.5 1.5]);
legend({'R1','R2','R3','R4','R5','R6','R7','R8'},'Location','Northwest','FontSize',labelS);
xlabel('x [meter]','FontSize',labelS); ylabel('y [meter]','FontSize',labelS)

% Tracking error
figure;
subplot(3,1,1) % x_d - x
plot(t,goal(1)-robotarium_data(5*i_atk-4,:),'LineWidth',lineW)
grid on; axis([0 60 -2.5 2.5]);
xlabel('t (sec)','FontSize',labelS); ylabel('x error (meter)','FontSize',labelS)

subplot(3,1,2) % y_d - y
plot(t,goal(2)-robotarium_data(5*i_atk-3,:),'LineWidth',lineW)
grid on; axis([0 60 -2.5 2.5]);
xlabel('t (sec)','FontSize',labelS); ylabel('y error (meter)','FontSize',labelS)

subplot(3,1,3) % theta_d - theta
plot(t,goal(3)-robotarium_data(5*i_atk-2,:),'LineWidth',lineW)
grid on;
xlabel('t (sec)','FontSize',labelS); ylabel('heading error (rad)','FontSize',labelS)

% % Test statistic
% figure;
% subplot(3,1,1) % S_x(k)
% plot(t,robotarium_data(5*i_atk-4,:),'LineWidth',lineW)
% grid on; axis([0 60 -1.5 1.5]);
% xlabel('t (sec)','FontSize',labelS); ylabel('S_{x}(k)','FontSize',labelS)
% 
% subplot(3,1,2) % S_y(k)
% plot(t,robotarium_data(5*i_atk-4,:),'LineWidth',lineW)
% grid on; axis([0 60 -1.5 1.5]);
% xlabel('t (sec)','FontSize',labelS); ylabel('S_{y}(k)','FontSize',labelS)
% 
% subplot(3,1,3) % S_theta(k)
% plot(t,robotarium_data(5*i_atk-4,:),'LineWidth',lineW)
% grid on; axis([0 60 -1.5 1.5]);
% xlabel('t (sec)','FontSize',labelS); ylabel('S_{\theta}(k)','FontSize',labelS)

% Trajectory of the agent with attack
% figure;
% plot(robotarium_data(5*i_atk-4,:),robotarium_data(5*i_atk-3,:),'LineWidth',lineW)
% grid on; axis([-1.5 1.5 -1.5 1.5]);
% legend({sprintf('R%d\n',i_atk)},'Location','Northwest','FontSize',labelS);
% xlabel('x [meter]','FontSize',labelS); ylabel('y [meter]','FontSize',labelS)