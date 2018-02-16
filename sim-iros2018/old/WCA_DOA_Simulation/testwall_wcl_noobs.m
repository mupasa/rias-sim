%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to estimate direction of angle (DOA)     %
%                 started coding this on 2012-07-23     %
%                                         by BCM        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global p
% 
clear psedo_angle_range
clear psedo_raw_dbm_data
clear psedo_raw_watt_data

PowTx = TXpower;
RealDoA = ACTangle; %degree
PhiRx = -RealDoA;
%PhiRx = 0;

SampleType = 4;
set_num_sample = [91, 37, 19, 13, 10,  7,  5, 4];
%set_num_sample = [91, 37, 19, 17, 10,  7,  5, 4]; %try -120 to 120
set_intervals =  [ 2,  5, 10, 15 ,20, 30, 45, 60];
num_sample = set_num_sample(SampleType);
intervals = set_intervals(SampleType);
theta = 90;
%theta = 120; %try -120 to 120
gamma1 = est_dis * 0.15; % determine vector size that indicate each sample

if sim == 1 %only single simulation is runinig
    % robot's heading
    plot([rx(1) (rx(1)-v1dot(1)*est_dis*0.20)] , [ry(1) (ry(1)-v1dot(2)*est_dis*0.20)] , ':k' ,'LineWidth',1)
end

for i=1:num_sample
PowerRX(i) = testwall_PowerRX_func(PowTx, PhiRx);  % need to change function inside when 120 range is set
%PowerRXn(i) = PowerRX(i) + (PowerRX(i)/5).*randn(1,1) %nomal distribution
PowerRXn(i) = ricernd(PowerRX(i),(PowerRX(i)/5)); %rician distribution % (PowerRX(i)/n)) n is std

tv(1) = gamma1*v1dot(1)*cosd(theta) - gamma1*v1dot(2)*sind(theta);
tv(2) = gamma1*v1dot(2)*cosd(theta) + gamma1*v1dot(1)*sind(theta);

if sim == 1 %only single simulation is runinig
    figure(1)
    plot([rx(1) rx(1)-tv(1)], [ry(1) ry(1)-tv(2)], '-sm' ,'LineWidth',1) % line from robot's
    pause(0.1)
    %pause
end
PhiRx = PhiRx + intervals;
theta = theta - intervals;
end
raw_dbm_data = round(-log10(PowerRXn)*10); %with recian factor effects (noise)

% To plot
gain = -raw_dbm_data;
figure2 = createStemPlot(gain); %stem plot
hold on

%polynomial fitting
x = 1:num_sample;
y = -gain;
p = polyfit(x,y,4);
f = polyval(p,x);
%x0 = 9; %120 degree
x0 = 7; %90 degree
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon =[];
lb = 1;
ub=13;%
%ub=17; %try -120 to 120
%options = optimset('LargeScale', 'off', 'Display', 'off'); 
options=optimset('Display', 'off','Algorithm','sqp');
%[xstar,fval] = fminunc(@myfun,x0,options);
%[xstar,fval] = fminsearch(@myfun,x0);
%[xstar,fval] = fmincon(@myfun,x0,AA,BB,options);       
[xstar,fval] = fmincon(@myfun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
EstDoA_fit = 15*xstar-105; %when 13 samples & 15 degree intervals
%EstDoA_fit = 15*xstar-135% try -120 to 120

%EstDoA_fit = 10*xstar(1)-100 %when 19 samples & 10 degree intervals
%EstDoA_fit = 2*xstar(1)-92 %when 91 samples & 2 degree intervals
plot(x,-f,'-r')
plot(xstar(1),-fval,'sg')
% %legend('Raw Data','Raw Data (Min Found)')
% 
figure3 = figure;

 angle_range = -90:intervals:90;
%angle_range = -120:intervals:120; % try -120 to 120

%polar(angle_range, gain, 's') 
%set(figure3, 'Position', [550 100 700 300]) %big size
set(figure3, 'Position',[650 100 350 200]) %small size
dirplot(angle_range, gain,'o-'); %polar plot
hold on;
dirplot(RealDoA, max(gain),'*'); %polar plot

%% Weighted Centroid Localization (WCL)
raw_watt_data = 10.^(-raw_dbm_data/10); %Convert dBm to watt
PT = sum(raw_watt_data); % sum of Multipath Power Gain, denominator
a = sum(angle_range .* raw_watt_data); %numerator
EstDoA_WCL = a / PT;
%plot(20, 320,'*'); %polar plot
dirplot(EstDoA_WCL, max(gain)-5,'xr'); %polar plot

%% WCL Upgrade

if EstDoA_WCL > 0

    %refPoint = round((EstDoA_fit+105)/15);
    refPoint = round((EstDoA_WCL+105)/15); %90
    %refPoint = round((EstDoA_WCL+135)/15); % try -120 to 120
    num_livePoint = num_sample - refPoint;
    psedo_raw_dbm_data = raw_dbm_data;
    for i=1:refPoint-(1+num_livePoint)
        psedo_raw_dbm_data(num_sample+i) = raw_dbm_data(refPoint-(i+num_livePoint));
    end
    psedo_raw_watt_data = 10.^(-psedo_raw_dbm_data/10); %Convert dBm to watt
    psedo_PT = sum(psedo_raw_watt_data); % sum of Multipath Power Gain, denominator
    psedo_angle_range = -90:intervals:(90+(refPoint-(1+num_livePoint))*intervals);
    %psedo_angle_range = -120:intervals:(120+(refPoint-(1+num_livePoint))*intervals); % try -120 to 120
    psedo_a = sum(psedo_angle_range .* psedo_raw_watt_data); %numerator
    psedo_EstDoA_WCL1 = psedo_a / psedo_PT;

else
    %refPoint = round((EstDoA_fit+105)/15);
    refPoint = round((EstDoA_WCL+105)/15); 
    %refPoint = round((EstDoA_WCL+135)/15); % try -120 to 120
    num_livePoint = num_sample - refPoint;
    
    for i=1: -2*refPoint+num_sample+1;
        psedo_raw_dbm_data(i) = raw_dbm_data(num_sample-(i-1));
    end
    for i=1:num_sample
        psedo_raw_dbm_data((-2*refPoint+num_sample+1)+i) = raw_dbm_data(i);
    end
    psedo_raw_watt_data = 10.^(-psedo_raw_dbm_data/10); %Convert dBm to watt
    psedo_PT = sum(psedo_raw_watt_data); % sum of Multipath Power Gain, denominator
    psedo_angle_range = -90+(refPoint-(1+num_livePoint))*intervals:intervals:90;
    %psedo_angle_range = -120+(refPoint-(1+num_livePoint))*intervals:intervals:120; % try -120 to 120
    psedo_a = sum(psedo_angle_range .* psedo_raw_watt_data); %numerator
    psedo_EstDoA_WCL1 = psedo_a / psedo_PT;
end

dirplot(psedo_EstDoA_WCL1, max(gain)-5,'+m'); %polar plot

%% to DISPLAY final estimation
gamma2 = est_dis * 0.25; % determine vector size that indicate final estimated DoA
theta = -psedo_EstDoA_WCL1;

% plot([rx(1) rx(1)-tv(1)], [ry(1) ry(1)-tv(2)], '-k' ,'LineWidth',2) %
% line from robot's
tv(1) = gamma2*v1dot(1)*cosd(theta) - gamma2*v1dot(2)*sind(theta); %*2 is used to double a vector size
tv(2) = gamma2*v1dot(2)*cosd(theta) + gamma2*v1dot(1)*sind(theta);

figure(1)
hold on
quiver(rx(1),ry(1),-tv(1),-tv(2),'-k','LineWidth',3)

disp(' ')
disp(['Actual DoA         =     ' num2str(ACTangle)])
disp(['Polyfit DoA        =     ' num2str(EstDoA_fit)])
disp(['WCL DoA            =     ' num2str(EstDoA_WCL)])
disp(['AWCL DoA           =     ' num2str(psedo_EstDoA_WCL1)])
disp(['Polyfit Err        =     ' num2str(abs(ACTangle-EstDoA_fit))])
disp(['WCL Err            =     ' num2str(abs(ACTangle-EstDoA_WCL))])
disp(['AWCL Err           =     ' num2str(abs(ACTangle-psedo_EstDoA_WCL1))])

disp(' ')
disp(['Robot Position     =     ' num2str(rx(1)) ' , ' num2str(ry(1))])
disp(['Robot Heading      =     ' num2str(RbAng)])
disp(['TX Power           =     ' num2str(TXpower)])
disp(['Actual dist        =     ' num2str(rea_dis)])
disp(['Estimated dist     =     ' num2str(est_dis)])