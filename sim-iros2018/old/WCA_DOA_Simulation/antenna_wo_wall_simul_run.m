%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to estimate DOA in no obstacle environment     %
%                 Last code updated on   2017-07-26           %
%                                         by BCM              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% You can determine single run or multiple runs by setting SIMUL_TYPE

clear; close all; clc; 

SIMUL_TYPE = 'single';  % for single run
%SIMUL_TYPE = 'multiple'; % for multiple run

nb_pts                   = 128;
nr                       = 3;

option.TX                = 0;
option.RX                = 0;
option.path              = 0;

L0                       = 20.2; %Reference loss value at 1m
n                        = 2;    %Power decay factor

%load  data_mw1; %without obstacle, open environment
load  map_open; %without obstacle, open environment
%load  data_mw4; %with obstacle, enclosed environment

flp                      = load_flp('map_open.flp');

%flp                      = load_flp('simple2.flp');


temp                     = flp.geom.planes([1 , 4 , 7] , :);
xmin                     = min(temp(:));
xmax                     = max(temp(:));


temp                     = flp.geom.planes([2 , 5 , 8] , :);
ymin                     = min(temp(:));
ymax                     = max(temp(:));


temp                     = flp.geom.planes([3 , 6 , 9] , :);
zmin                     = min(temp(:));
zmax                     = max(temp(:));


vectx                    = (xmin:(xmax-xmin)/(nb_pts-1):xmax);
vecty                    = (ymin:(ymax-ymin)/(nb_pts-1):ymax);


[X , Y]                  = meshgrid(vectx , vecty);
Z                        = ((zmax-zmin)/2)*ones(nb_pts , nb_pts);
RX                       = [X(:) , Y(:) , Z(:)]';
figure1= figure;
plot_flp(flp , option);

title('click to select wifi source position')

hold on

% [TxPosx , TxPosy]                 = getpts;
% TxPosx = 22;
% TxPosy = 20;
 TxPosx = 7.4605;
 TxPosy = 37.4605;
temp                    = (zmax-zmin)/2;
flp.info.TXpoint        = [TxPosx' ; TxPosy' ; temp(: , ones(1 , length(TxPosx)))];

% flp.info.TXpoint         = [6000 ; 6000 ; 500];

TXpoint                 = [TxPosx' ; TxPosy'];
%plot(flp.info.TXpoint(1 , :) , flp.info.TXpoint(2 , :) , 'c*');
plot(TXpoint(1 , :) , TXpoint(2 , :) , 'k+' , 'markersize' , 10)
drawnow


rs_amp                  = total_power3(flp.info.TXpoint , RX , flp.geom.planes , flp.geom.material , flp.info.fc , nr);
rs_amp = rs_amp*0.5;
%rs_amp = rs_amp;
tp = 20*log10(reshape(sum(rs_amp , 1) , nb_pts, nb_pts));

figure(1)
imagesc(vectx , vecty , 20*log10(reshape(sum(rs_amp , 1) , nb_pts, nb_pts)));
hold on
plot_flp(flp);
%title(sprintf('top view (TxPosx - TxPosy), nr = %d',nr))
%title('three dimensional view')
xlabel('East (m)')
ylabel('North (m)')
zlabel('z in meters')
axis xy
colorbar
axis equal

title('DOA estimation with directional antenna')

%%
if(strcmp(SIMUL_TYPE,'single'))
  sim = 1;
  testwall_singleRun %for single run
else
  sim = 2;  %simulation 'multiple'
  testwall_multipleRun %for multiple run with vector plot
end

