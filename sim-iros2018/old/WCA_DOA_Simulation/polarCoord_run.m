%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to estimate direction of angle (DOA)           %  
% in polor coordinate in 180 degree                           %
%                 Last code updated on   2017-07-26           %
%                                         by BCM              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear all
close all
global p
PowTx = 200^3;
Dist=20e1; % meter
RealDoA = 80; %degree
PhiRx = -RealDoA;

SampleType = 4;
% set_num_sample = [91, 37, 19, 13, 10,  7,  5, 4];
% set_intervals =  [ 2,  5, 10, 15 ,20, 30, 45, 60];

set_num_sample = [91, 37, 19, 17, 10,  7,  5, 4]; %try -120 to 120
set_intervals =  [ 2,  5, 10, 15 ,20, 30, 45, 60];

num_sample = set_num_sample(SampleType);
intervals = set_intervals(SampleType);

for i=1:num_sample
PowerRX(i) = PowerRX_func(PowTx, PhiRx, Dist);
%PowerRXn(i) = PowerRX(i) + (PowerRX(i)/5).*randn(1,1) %nomal distribution
PowerRXn(i) = ricernd(PowerRX(i),(PowerRX(i)/5)); %rician distribution % (PowerRX(i)/n)) n is std
radPattern(i) = PhiRx;
PhiRx = PhiRx + intervals;
end

%raw_dbm_data = round(-log10(PowerRX)*10); %without recian factor effects(noise)
raw_dbm_data = round(-log10(PowerRXn)*10); %with recian factor effects (noise)

%raw_dbm_data = [27, 25, 19, 13, 13, 11, 11, 13, 13, 13, 17, 23, 29];
%raw_dbm_data = [13,13,13,11,13,13,19,25,31,33,33,35,33];
%raw_dbm_data = [27, 23, 17, 13, 13, 11, 11, 11, 13, 15, 17, 23, 29];
% raw_watt_data = [0.001995262,0.005011872,0.019952623,0.050118723,0.050118723,0.079432823,0.079432823,...
% 0.079432823,0.050118723,0.031622777,0.019952623,0.005011872,0.001258925];
%gain = log10(raw_watt_data)*10;

% To plot
gain = -raw_dbm_data;
createStemPlot(gain); %stem plot
hold on
%polynomial fitting
x = 1:num_sample;
y = -gain;
p = polyfit(x,y,4);
f = polyval(p,x);
x0 = 5;
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon =[];
lb = 1;

% ub=13;
ub=17; % try -120 to 120

%options = optimset('LargeScale', 'off', 'Display', 'off'); 
options=optimset('Display', 'off','Algorithm','sqp');
%[xstar,fval] = fminunc(@myfun,x0,options);
%[xstar,fval] = fminsearch(@myfun,x0);
%[xstar,fval] = fmincon(@myfun,x0,AA,BB,options);       
[xstar,fval] = fmincon(@myfun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

%EstDoA_fit = 15*xstar-105 %when 13 samples & 15 degree intervals
EstDoA_fit = 15*xstar-135% try -120 to 120

%EstDoA_fit = 10*xstar(1)-100 %when 19 samples & 10 degree intervals
%EstDoA_fit = 2*xstar(1)-92 %when 91 samples & 2 degree intervals
plot(x,-f,'-r')
plot(xstar(1),-fval,'sg')
% %legend('Raw Data','Raw Data (Min Found)')
% 
figure2 = figure;
% angle_range = -90:intervals:90;
angle_range = -120:intervals:120; % try -120 to 120

%polar(angle_range, gain, 's') 
%set(figure2, 'Position', [550 100 700 300]) %big size
set(figure2, 'Position',[650 100 350 200]) %small size
dirplot(angle_range, gain,'o-'); %polar plot
hold on;
dirplot(RealDoA, max(gain),'s'); %polar plot

%% Weighted Centroid Localization (WCL)
raw_watt_data = 10.^(-raw_dbm_data/10); %Convert dBm to watt
PT = sum(raw_watt_data); % sum of Multipath Power Gain, denominator
a = sum(angle_range .* raw_watt_data); %numerator
EstDoA_WCL = a / PT
%plot(20, 320,'*'); %polar plot
dirplot(EstDoA_WCL, max(gain)-5,'*'); %polar plot

