% Robotarium Test Data Plot
% Sangjun Lee
% 2/19/2018
clear; close all; clc;

% Import Data
load 2_26_2018_rundata.mat
[lim_row, lim_col] = size(data.pose);
N = data.N;
disp(data)

% General Settings
lineW = 1.3;
axisS = 15;
labelS = 17;

t_test = 45; % Actual test time (sec)
t = linspace(0,t_test,lim_row/3);

% Trajectory of all agents
figure;
hold on
for k = 1:N
    plot(data.pose(1:3:lim_row,k),data.pose(2:3:lim_row,k),'LineWidth',lineW);
    plot(data.pose(end-2,k),data.pose(end-1,k),'b^','LineWidth',lineW)
    text(data.pose(end-2,k)+0.03,data.pose(end-1,k)+0.03,...
        sprintf('%d',k),'FontSize',labelS)
end
plot(data.pose(1:3:lim_row,data.att_agent(1)),...
    data.pose(2:3:lim_row,data.att_agent(1)),'m','LineWidth',lineW);
plot(data.pose(1:3:lim_row,data.att_agent(2)),...
    data.pose(2:3:lim_row,data.att_agent(2)),'g','LineWidth',lineW);
plot(data.pose(1:3:lim_row,data.att_agent(3)),...
    data.pose(2:3:lim_row,data.att_agent(3)),'k','LineWidth',lineW);
plot(data.pose(end-2,data.att_agent),data.pose(end-1,data.att_agent),...
        'r^','LineWidth',lineW)
hold off;
grid on; axis([-1 1 -1 1]); ax = gca; ax.FontSize = axisS;
xlabel('x (m)','FontSize',labelS); ylabel('y (m)','FontSize',labelS)

% Tracking error of all agents
figure;
for i = 1:N
subplot(2,1,1) % x_d - x
plot(t,data.goal(1,i)-data.pose(1:3:lim_row,i),'LineWidth',lineW)
hold on; grid on; axis([0 45 -2 2]); ax = gca; ax.FontSize = axisS;
ylabel('x error (m)','FontSize',labelS)

subplot(2,1,2) % y_d - y
plot(t,data.goal(2,i)-data.pose(2:3:lim_row,i),'LineWidth',lineW)
hold on; grid on; axis([0 45 -2 2]); ax = gca; ax.FontSize = axisS;
xlabel('time (s)','FontSize',labelS); ylabel('y error (m)','FontSize',labelS)
end
hold off