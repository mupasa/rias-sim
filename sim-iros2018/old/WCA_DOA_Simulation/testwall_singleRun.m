
[rx , ry] = getpts;
%rx = [29.9681;29.9681];
%ry = [4.4175;14.9538];
ROpoint                 = [rx(1)' ; ry(1)'];
ROdir                 = [rx' ; ry'];

figure(1)
hold on
plot(ROpoint(1 , :) , ROpoint(2 , :) , 'k*' , 'markersize' , 10)
% plot(ROdir(1 , :) , ROdir(2 , :) , ':k' ,'LineWidth',1)
plot([rx(1) TxPosx], [ry(1) TxPosy], '-.w' ,'LineWidth',1) % line from robot's
%position to Tx
hold on
[icx,vcx] = searchclosest(vectx,rx(1));
[icy,vcy] = searchclosest(vecty,ry(1));
posn(1) = icx;
posn(2) = icy;
yy = rx(2) - rx(1);
xx = ry(2) - ry(1);
TXpower = tp(icy,icx); % TX Power

[posn(3), r] = cart2pol(xx,yy);
RbAng = radtodeg(posn(3)); % showing robot's current angle in degree

% figure
% contourf(tp(1:128,1:128),'DisplayName','mapbin(1:128,1:128)');figure(gcf)
% colorbar

v1=([rx(1),ry(1)]-[rx(2),ry(2)]);  %/2 is used for reduce vector size to its half.
v2=[rx(1),ry(1)]-[TxPosx,TxPosy];

% unit vector to TXpoint from robot
scalar_tx = sqrt(v2(1)^2+v2(2)^2);
v2dot=v2/scalar_tx;

% unit vector to current robot's heading
scalar_hd = sqrt(v1(1)^2+v1(2)^2);
v1dot=v1/scalar_hd;

%plot([rx(1) (rx(1)-v2dot(1)*5)], [ry(1) (ry(1)-v2dot(2)*5)], '-b' ,'LineWidth',1)

a1 = atan2(det([v1;v2;]),dot(v1,v2));
ACTangle = radtodeg(a1)*-1;

%% To determine if there are any intersection (to determine if NOS exists or not)
intsec = zeros(1,length(walls));
x1 = rx(1);
y1 = ry(1);

x2 = TxPosx;
y2 = TxPosy;

for i = 1: length(walls)
    x3 = walls(1,i);
    y3 = walls(2,i);

    x4 = walls(3,i);
    y4 = walls(4,i);
    intsec(i) = det([1,1,1;x1,x2,x3;y1,y2,y3])*det([1,1,1;x1,x2,x4;y1,y2,y4]) <= 0 &&...
        det([1,1,1;x1,x3,x4;y1,y3,y4])*det([1,1,1;x2,x3,x4;y2,y3,y4]) <= 0;
end
% if any two lines are intersected, then sum(intsec) will be larger than 0
% for example, if there is one intersection with the last line, 
% then intsec becomes [0 0 0 0 0 1], and sum(intsec) becomes 1.
bool_intsec = sum(intsec) > 0;

% estimate distance
rea_dis = sqrt((TXpoint(1)-rx(1))^2 + (TXpoint(2) - ry(1))^2); %calculate distance Tx and Rx

if bool_intsec == 1
%if -TXpower >= 50
    %est_dis = 10
    est_dis = sqrt(exp((-TXpower*5-L0)/10))/200;
    est_dis = rea_dis;
else
    est_dis = sqrt(exp((-TXpower*5-L0)/10));
    est_dis = rea_dis;
end

%% save figures to file
%mkdir([date '-S1']) %run once a day to create folder to save figures
% cc = clock;
% mkdir(['S' num2str(cc(1)) '-' num2str(cc(2)) '-' num2str(cc(3)) '-'...
%     num2str(cc(4)) '-' num2str(cc(5)) '-' num2str(round(cc(6)))]);
% 
% fpat=(['S' num2str(cc(1)) '-' num2str(cc(2)) '-' num2str(cc(3)) '-'...
%     num2str(cc(4)) '-' num2str(cc(5)) '-' num2str(round(cc(6)))]);

%save initial map figure
% fnam=['mapBefore'];
% saveas(figure1,[fpat,filesep,fnam],'fig');  %save map figure
% saveas(figure1,[fpat,filesep,fnam],'eps');  %save map figure
% saveas(figure1,[fpat,filesep,fnam],'emf');  %save map figure

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if there is an intersection, call to "testwall_wcl_noobs.m" that
% generates "raw_dbm_data" by integrating all signal strength from each ray. 
% If not, call to "testwall_wcl_withobs.m" that generates "raw_dbm_data" by
% TXpower and radio pattern.

if bool_intsec == 1
    testwall_wcl_withobs
else
    testwall_wcl_noobs
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%save final map figure
% fnam=['mapAfter'];
% saveas(figure(1),[fpat,filesep,fnam],'fig');  
% saveas(figure(1),[fpat,filesep,fnam],'eps');  
% saveas(figure(1),[fpat,filesep,fnam],'emf');  
% 
% fnam1=['stemPlot'];
% saveas(figure2,[fpat,filesep,fnam1],'fig');        
% saveas(figure2,[fpat,filesep,fnam1],'eps');  
% saveas(figure2,[fpat,filesep,fnam1],'emf');  
% fnam2=['polarPlot'];
% saveas(figure3,[fpat,filesep,fnam2],'fig');           
% saveas(figure3,[fpat,filesep,fnam2],'eps');  
% saveas(figure3,[fpat,filesep,fnam2],'emf');  

%% save variables to txt files
%save estimation history
DoA_name = ['Actual DoA         =     ';...
            'Polyfit DoA        =     ';...
            'WCL DoA            =     ';...
            'AWCL DoA           =     ';...
            'Polyfit Err        =     ';...
            'WCL Err            =     ';...
            'AWCL Err           =     '];
        
DoA_data = [ACTangle EstDoA_fit, EstDoA_WCL, psedo_EstDoA_WCL1,...
    abs(ACTangle-EstDoA_fit), abs(ACTangle-EstDoA_WCL), abs(ACTangle-psedo_EstDoA_WCL1)];
% fileID=fopen([fpat '\DoA_History.txt'],'at');
% for i=1:length(DoA_data)
% fprintf(fileID,'%s %3.4f\n',DoA_name(i,:), DoA_data(i));
% end
% fclose(fileID);

%save robot's history
Robot_name = ['Robot x-position   =     ';...
              'Robot y-position   =     ';...
              'Robot Heading      =     ';...
              'TX Power           =     ';...
              'Actual dist        =     ';...
              'Estimated dist     =     '];
          
Robot_data = [rx(1), ry(1), RbAng, TXpower, rea_dis, est_dis];
% fileID=fopen([fpat '\Robot_History.txt'],'at');
% for i=1:length(Robot_data)
% fprintf(fileID,'%s %3.4f\n',Robot_name(i,:), Robot_data(i));
% end
% fclose(fileID);

