%% get size of course
% 
% mapbin = zeros([128,128])+255;
% mapbin(:,1) = 0;
% mapbin(:,128) = 0;
% mapbin(1,:) = 0;
% mapbin(128,:) = 0;
% % 
% % mapbin(1:33,73) = 0; % for 44 by 44 size
% % mapbin(96:128,73) = 0; %for 44 by 44 size
% 
% mapbin(1:46,73) = 0; % for 44 by 17 size
% mapbin(83:128,73) = 0; %for 44 by 17 size

SampleType = 4;
%set_num_sample = [91, 37, 19, 13, 10,  7,  5, 4];
set_num_sample = [91, 37, 19, 17, 10,  7,  5, 4]; %try -120 to 120
set_intervals =  [ 2,  5, 10, 15 ,20, 30, 45, 60];
num_sample = set_num_sample(SampleType);
intervals = set_intervals(SampleType);

%%
% load  mapbin4; %mapbin3 means binary map for squred-map 44x44, two walls vertically


mapbin = zeros([128,128])+255;
mapbin(:,1) = 0;
mapbin(:,128) = 0;
mapbin(1,:) = 0;
mapbin(128,:) = 0;

mapbin(40:48,1:88) = 0; %real
%mapbin(40:44:48,1:88) = 0; %real
%mapbin(48,1:88) = 0;

mapbin(69:77,44:128) = 0; %real
%mapbin(69:73:77,44:128) = 0; %real
%apbin(77,44:128) = 0;

[dimy, dimx] = size(tp);
clear gain
clear ogain
%for 100 steps.
loss_fac = 0;
%intTheta = 90;
intTheta = 120; %try -120 to 120
gamma1 = est_dis * 0.15; % determine vector size that indicate each sample
if sim == 1 %only single simulation is runinig
    % robot's heading
    plot([rx(1) (rx(1)-v1dot(1)*est_dis*0.20)] , [ry(1) (ry(1)-v1dot(2)*est_dis*0.20)] , ':k' ,'LineWidth',1)
end

%for k = 1 : 13
for k = 1 : 17 %try -120 to 120
    %cur_scan_ang = (15*k-105);
    cur_scan_ang = (15*k-135); %try -120 to 120
    for j = 1: 17 % number of beam, n = 15 degree
        for i = 1:181
            aa = radtodeg(posn(3));
            %aa = aa + (5*j-35) + (15*k-105); % -30 - +30
            %aa = aa + (5*j-45) + (15*k-105); % -40 - +40
            bb = (intervals*k -((num_sample*intervals-intervals)/2+intervals));
            aa = aa + (5*j-45) + bb; %try -120 to 120
            %aa = aa + (15*j-45) + (15*k-105); % beamwidth =  [-30,-15,-,15,30] , scanrange[-90,-75,-60...60,75,90]
            bb = degtorad(aa);
            %get x&y coords of point to test
            antsx = posn(1)+ i*sin(bb);
            antsy = posn(2)+ i*cos(bb);
            %gain(i,j) = tp(round(antsy), round(antsx)) * (rea_dis*0.045+0.10);
            ogain(i,j) = tp(round(antsy), round(antsx));
            %gain(i,j) = tp(round(antsy), round(antsx));
            %phi = (5*j-35);
            phi = (5*j-45); %-40 to +40

            if  -40 <= phi && phi <= 40
                %RxAntennaGain = cosd(phi).^2; % when BW =50 degree
                RxAntennaGain = cosd(phi).^2; % when BW =30 degree    
%             elseif -90 >= phi || 90 <= phi
%                 RxAntennaGain = 0.00875;
            else
                RxAntennaGain = cosd(phi).^4;
            end
            %gain(i,j) = tp(round(antsy), round(antsx)) + loss_fac*0.5;
            gain(i,j) = ogain(i,j) / RxAntennaGain;
            % gain(i,j) = tp(round(antsy), round(antsx)) + loss_fac*0.5;

        %     %see if the course at x & y is an obstacle or not
            if(mapbin(round(antsy),round(antsx)) == 0)
                %if yes, return the distance to the obstacle
                %dist = i-rad;
                break;
            elseif(0 < mapbin(round(antsy),round(antsx)) && mapbin(round(antsy),round(antsx)) <= 10)
                %loss_fac = mapbin(round(antsy),round(antsx));
                loss_fac = 0;
            end 
        end
    end

  
    sv(1) = gamma1*v1dot(1)*cosd(intTheta) - gamma1*v1dot(2)*sind(intTheta);
    sv(2) = gamma1*v1dot(2)*cosd(intTheta) + gamma1*v1dot(1)*sind(intTheta); 
    intTheta = intTheta - 15;
    if sim == 1 %only single simulation is runinig    
        figure(1)
        plot([rx(1) rx(1)-sv(1)], [ry(1) ry(1)-sv(2)], '-sm' ,'LineWidth',1) % line from robot's
    end
    pp = gain(:,:);
    n = sum(pp~=0);
    n(n==0) = NaN;
    m = sum(pp) ./ n;    

    pp(~pp) = nan;
    col_mins = max(pp);
%         k
%     pause
   % max(max(pp))
    if bool_intsec == 1    
    %if -TXpower >= 50 
        %this means a robot currently lies in non-LOS region. Thus, I add m
        for c=1:17
                leng_pp_col = length(pp(isfinite(pp(:, c)), c));
                pp_col = pp(1:leng_pp_col,c);
                w = (pp_col+100).^50;
                sample_mean_col(c)= wmean(pp_col,w);
        end
        % sample_mean_col = nanmean(pp);
        gain_col = sample_mean_col;
        gain_w = (gain_col+100).^50;
        cur_rssi(k) = wmean(gain_col,gain_w);  
        %cur_rssi(k) = max(max(pp));
    else
        for c=1:17
                leng_pp_col = length(pp(isfinite(pp(:, c)), c));
                pp_col = pp(1:leng_pp_col,c);
                w = (pp_col+100).^30;
                sample_mean_col(c)= wmean(pp_col,w);
        end
        % sample_mean_col = nanmean(pp);
        gain_col = sample_mean_col;
        gain_w = (gain_col+100).^30;
        cur_rssi(k) = wmean(gain_col,gain_w);    
    end
    clear pp_col;
    clear w;
    clear gain
    clear ogain
end
% %if no obstacles found within 100 pixels, return 100
% dist = 100;
% return;
[qq,ww]=max(cur_rssi);
%bestang=(15*ww-105);
bestang=(15*ww-135); %try -120 to 120

%%
global p
% 
clear psedo_angle_range
clear psedo_raw_dbm_data
clear psedo_raw_watt_data

PowTx = TXpower;
RealDoA = ACTangle; %degree
PhiRx = -RealDoA;
%PhiRx = 0;

%%%%%%%%%%%%%I copied these above %%%%%%%%%%%%%
% SampleType = 4;
% %set_num_sample = [91, 37, 19, 13, 10,  7,  5, 4];
% set_num_sample = [91, 37, 19, 17, 10,  7,  5, 4]; %try -120 to 120
% set_intervals =  [ 2,  5, 10, 15 ,20, 30, 45, 60];
% num_sample = set_num_sample(SampleType);
% intervals = set_intervals(SampleType);

%to shift up dbm overall (around 40dBm), I add an artificial constant with TXpower.
%T1 = (-TXpower/2);
%T1 = -TXpower^3/2500;
T1 = 0;
raw_dbm_data = -(cur_rssi)+ T1; 
raw_dbm_data = round(ricernd(raw_dbm_data,(raw_dbm_data/300))); %with recian factor effects (noise)


% To plot
T2 = -15; %to pull up all dBm
gain = -raw_dbm_data+T2;
figure2 = createStemPlot(gain); %stem plot
hold on
%polynomial fitting
x = 1:num_sample;
y = -gain;
p = polyfit(x,y,4);
f = polyval(p,x);
x0 = 9;
A = [];
b = [];
Aeq = [];
beq = [];
nonlcon =[];
lb = 1;
%ub=13;
ub=17; %try -120 to 120
%options = optimset('LargeScale', 'off', 'Display', 'off'); 
options=optimset('Display', 'off','Algorithm','sqp');
%[xstar,fval] = fminunc(@myfun,x0,options);
%[xstar,fval] = fminsearch(@myfun,x0);
%[xstar,fval] = fmincon(@myfun,x0,AA,BB,options);       
[xstar,fval] = fmincon(@myfun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
%EstDoA_fit = 15*xstar-105; %when 13 samples & 15 degree intervals
EstDoA_fit = 15*xstar-135; %%try -120 to 120

%EstDoA_fit = 10*xstar(1)-100 %when 19 samples & 10 degree intervals
%EstDoA_fit = 2*xstar(1)-92 %when 91 samples & 2 degree intervals
plot(x,-f,'-r')
plot(xstar(1),-fval,'sg')
% %legend('Raw Data','Raw Data (Min Found)')
% 
figure3 = figure;
%angle_range = -90:intervals:90;
angle_range = -120:intervals:120; %try -120 to 120

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
    %refPoint = round((EstDoA_WCL+105)/15);
    refPoint = round((EstDoA_fit+135)/15); % for Hybrid
    %refPoint = round((EstDoA_WCL+135)/15); %try -120 to 120
    num_livePoint = num_sample - refPoint;
    psedo_raw_dbm_data = raw_dbm_data;
    for i=1:refPoint-(1+num_livePoint)
        psedo_raw_dbm_data(num_sample+i) = raw_dbm_data(refPoint-(i+num_livePoint));
    end
    psedo_raw_watt_data = 10.^(-psedo_raw_dbm_data/10); %Convert dBm to watt
    psedo_PT = sum(psedo_raw_watt_data); % sum of Multipath Power Gain, denominator
    %psedo_angle_range = -90:intervals:(90+(refPoint-(1+num_livePoint))*intervals);
    psedo_angle_range = -120:intervals:(120+(refPoint-(1+num_livePoint))*intervals); % try -120 to 120
    psedo_a = sum(psedo_angle_range .* psedo_raw_watt_data); %numerator
    psedo_EstDoA_WCL1 = psedo_a / psedo_PT;

else
    %refPoint = round((EstDoA_fit+105)/15);
    %refPoint = round((EstDoA_WCL+105)/15);
    refPoint = round((EstDoA_fit+135)/15); % for Hybrid
    %refPoint = round((EstDoA_WCL+135)/15); %try -120 to 120
    num_livePoint = num_sample - refPoint;
    
    for i=1: -2*refPoint+num_sample+1;
        psedo_raw_dbm_data(i) = raw_dbm_data(num_sample-(i-1));
    end
    for i=1:num_sample
        psedo_raw_dbm_data((-2*refPoint+num_sample+1)+i) = raw_dbm_data(i);
    end
    psedo_raw_watt_data = 10.^(-psedo_raw_dbm_data/10); %Convert dBm to watt
    psedo_PT = sum(psedo_raw_watt_data); % sum of Multipath Power Gain, denominator
    %psedo_angle_range = -90+(refPoint-(1+num_livePoint))*intervals:intervals:90;
    psedo_angle_range = -120+(refPoint-(1+num_livePoint))*intervals:intervals:120;% try -120 to 120
    psedo_a = sum(psedo_angle_range .* psedo_raw_watt_data); %numerator
    psedo_EstDoA_WCL1 = psedo_a / psedo_PT;
end

dirplot(psedo_EstDoA_WCL1, max(gain)-5,'+m'); %polar plot

%% to DISPLAY
gamma2 = est_dis * 0.25; % determine vector size that indicate final estimated DoA
theta = -psedo_EstDoA_WCL1;
%theta = -EstDoA_WCL;
% theta = -EstDoA_fit;

% plot([rx(1) rx(1)-tv(1)], [ry(1) ry(1)-tv(2)], '-k' ,'LineWidth',2) %
% line from robot's
tv(1) = v1dot(1)*gamma2*cosd(theta) - v1dot(2)*gamma2*sind(theta); %*2 is used to double a vector size
tv(2) = v1dot(2)*gamma2*cosd(theta) + v1dot(1)*gamma2*sind(theta);

figure(1)
hold on
quiver(rx(1),ry(1),-tv(1),-tv(2),'-k','LineWidth',2)

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