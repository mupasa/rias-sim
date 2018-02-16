ROpoint                 = [rx(1)' ; ry(1)'];
rx(2) = TxPosx;
ry(2) = TxPosy;
ROdir                 = [rx' ; ry'];


[icx,vcx] = searchclosest(vectx,rx(1));
[icy,vcy] = searchclosest(vecty,ry(1));
posn(1) = icx;
posn(2) = icy;
yy = rx(2) - rx(1);
xx = ry(2) - ry(1);
TXpower = tp(icy,icx); % TX Power

[posn(3), r] = cart2pol(xx,yy);
RbAng = radtodeg(posn(3)); % showing robot's current angle in degree

v1=([rx(1),ry(1)]-[rx(2),ry(2)]);  %/2 is used for reduce vector size to its half.
v2=[rx(1),ry(1)]-[TxPosx,TxPosy];

% unit vector to TXpoint from robot
scalar_tx = sqrt(v2(1)^2+v2(2)^2);
v2dot=v2/scalar_tx;

% unit vector to current robot's heading
scalar_hd = sqrt(v1(1)^2+v1(2)^2);
v1dot=v1/scalar_hd;

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

%% save variables to txt files
%DoA history
fileID=fopen([fpat '\Actual_DoA.txt'],'a');
fprintf(fileID,'%3.4f\n',ACTangle);
fclose(fileID);

fileID=fopen([fpat '\Polyfit_DoA.txt'],'a');
fprintf(fileID,'%3.4f\n',EstDoA_fit);
fclose(fileID);

fileID=fopen([fpat '\WCL_DoA.txt'],'a');
fprintf(fileID,'%3.4f\n',EstDoA_WCL);
fclose(fileID);

fileID=fopen([fpat '\AWCL_DoA.txt'],'a');
fprintf(fileID,'%3.4f\n',psedo_EstDoA_WCL1);
fclose(fileID);

fileID=fopen([fpat '\Polyfir_Err.txt'],'a');
fprintf(fileID,'%3.4f\n',abs(ACTangle-EstDoA_fit));
fclose(fileID);

fileID=fopen([fpat '\WCL_Err.txt'],'a');
fprintf(fileID,'%3.4f\n',abs(ACTangle-EstDoA_WCL));
fclose(fileID);

fileID=fopen([fpat '\AWCL_Err.txt'],'a');
fprintf(fileID,'%3.4f\n',abs(ACTangle-psedo_EstDoA_WCL1));
fclose(fileID);

% robot history
fileID=fopen([fpat '\Ro_Posx.txt'],'a');
fprintf(fileID,'%3.4f\n',rx(1));
fclose(fileID);

fileID=fopen([fpat '\Ro_Posy.txt'],'a');
fprintf(fileID,'%3.4f\n',ry(1));
fclose(fileID);

fileID=fopen([fpat '\Ro_Head.txt'],'a');
fprintf(fileID,'%3.4f\n',RbAng);
fclose(fileID);

fileID=fopen([fpat '\TX_Power.txt'],'a');
fprintf(fileID,'%3.4f\n',TXpower);
fclose(fileID);

fileID=fopen([fpat '\Act_Dist.txt'],'a');
fprintf(fileID,'%3.4f\n',rea_dis);
fclose(fileID);

fileID=fopen([fpat '\Est_Dist.txt'],'a');
fprintf(fileID,'%3.4f\n',est_dis);
fclose(fileID);


