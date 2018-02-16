
%% save figures to file
%mkdir([date '-S1']) %run once a day to create folder to save figures
cc = clock;
mkdir(['M' num2str(cc(1)) '-' num2str(cc(2)) '-' num2str(cc(3)) '-'...
    num2str(cc(4)) '-' num2str(cc(5)) '-' num2str(round(cc(6)))]);

fpat=(['M' num2str(cc(1)) '-' num2str(cc(2)) '-' num2str(cc(3)) '-'...
    num2str(cc(4)) '-' num2str(cc(5)) '-' num2str(round(cc(6)))]);

%save initial map figure
fnam=['mapBefore'];
saveas(figure1,[fpat,filesep,fnam],'fig');  %save map figure
saveas(figure1,[fpat,filesep,fnam],'eps');  %save map figure
saveas(figure1,[fpat,filesep,fnam],'emf');  %save map figure
%%

for ii=1:9    % y-coordinate
    for jj=1:9 % x-coordinate
        %differ robot's position
%         rx(1) = (jj)*5 + 1;
%         ry(1) = (ii)*5 + 1;
        
        rx(1) = (jj-1)*5+2;
        ry(1) = (ii-1)*5+2;
        
        %call multiple run function
        testwall_multiple_run_func
    
        %save stem plot and polar plot to a date folder
        fnam1=['stemPlot_' num2str(ii) '_' num2str(jj)];
        saveas(figure2,[fpat,filesep,fnam1],'fig');        
        saveas(figure2,[fpat,filesep,fnam1],'eps');  
        saveas(figure2,[fpat,filesep,fnam1],'emf');  
        fnam2=['polarPlot_' num2str(ii) '_' num2str(jj)];
        saveas(figure3,[fpat,filesep,fnam2],'fig');           
        saveas(figure3,[fpat,filesep,fnam2],'eps');  
        saveas(figure3,[fpat,filesep,fnam2],'emf');  
        
        %close current opened figures to show only last figures
        close(figure2)
        close(figure3)
    end
end

%save final map figure
fnam=['map_after'];
saveas(figure(1),[fpat,filesep,fnam],'fig');  
saveas(figure(1),[fpat,filesep,fnam],'eps');  
saveas(figure(1),[fpat,filesep,fnam],'emf');  

%save command window
fnam=['data_history.txt'];
diary([fpat,filesep,fnam])
diary('off'); 

%open two plots that have been saved lastsly.
open([fpat '\stemPlot_' num2str(ii) '_' num2str(jj) '.fig'])
open([fpat '\polarPlot_' num2str(ii) '_' num2str(jj) '.fig'])

