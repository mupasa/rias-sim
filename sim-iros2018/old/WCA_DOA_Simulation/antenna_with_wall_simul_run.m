%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This code is to estimate DOA in an obstacle environment     %
%                 Last code updated on   2017-07-26           %
%                                         by BCM              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% single and multiple run
% this file only generates map and TX power propagation whenever wall exists or
% not.
% antenna_simul_run 
%  - > testwall_single_run 
%       -> testwall_wcl_withobs
%       -> testwall_wcl_noobs
%
% antenna_multiple_run 

% You can determine single run or multiple runs by setting SIMUL_TYPE

close all
clear all
clc

%SIMUL_TYPE = 'single';  % for single run
SIMUL_TYPE = 'multiple'; % for single run


L0                       = 20.2; %Reference loss value at 1m
n                        = 2;    %Power decay factor

nb_pts                   = 128;  %Number of steps to grid TxPosx,TxPosy

%load  data_mw1; %without obstacle, open environment
load  data_mw4; %with obstacle, enclosed environment

x_min                    = min(min(walls([1 , 3] , :) , [] , 2));
x_max                    = max(max(walls([1 , 3] , :) , [] , 2));
y_min                    = min(min(walls([2 , 4] , :) , [] , 2));
y_max                    = max(max(walls([2 , 4] , :) , [] , 2));
vectx                    = (x_min:(x_max-x_min)/(nb_pts-1):x_max);
vecty                    = (y_min:(y_max-y_min)/(nb_pts-1):y_max);
[X , Y]                  = meshgrid(vectx , vecty);
RXpoint                  = [X(:) , Y(:)]';

offset                   = 0.5;

figure1= figure;
axis([x_min-offset x_max+offset y_min-offset y_max+offset])
plot(walls([1 , 3] , :) , walls([2 , 4] , :) , 'linewidth' , 2);
title('click to select wifi source position')
axis equal

hold on
%%%%%%%%%%%% Input to position of transmitters %%%%%%%%%%
[TxPosx , TxPosy]                 = getpts;
% TxPosx = 40;
% TxPosy = 6;
%%%%%%%%%%%% Input to position of transmitters %%%%%%%%%%

TXpoint                 = [TxPosx' ; TxPosy'];
%  text(sum(walls([1 , 3] , :))/2 , sum(walls([2 , 4] , :))/2 , num2str((1:size(walls,2))','%d') , 'fontsize' , 6)
plot(TXpoint(1 , :) , TXpoint(2 , :) , 'k+' , 'markersize' , 10)
hold off
drawnow

rs_amp                   = multiwall_model(TXpoint ,RXpoint , walls , material , L0 , n);
P                        = log(sum(exp(rs_amp) , 1));
tp = reshape(P , nb_pts, nb_pts)/5;
%load tp_data1;

figure(1)
imagesc(vectx , vecty , tp);
hold on
plot(walls([1 , 3] , :) , walls([2 , 4] , :) , 'linewidth' , 2);
plot(TXpoint(1 , :) , TXpoint(2 , :) , 'k+' , 'markersize' , 10)

%hold off
axis xy
colorbar
axis equal
%title(sprintf('Multiwall model, L0 = %4.2f, n = %4.2f' , L0 , n))
title('wifi source localization with directional antenna')

%%
if(strcmp(SIMUL_TYPE,'single'))
  sim = 1;
  testwall_singleRun %for single run
else
  sim = 2;  %simulation 'multiple'
  testwall_multipleRun %for multiple run with vector plot
end




