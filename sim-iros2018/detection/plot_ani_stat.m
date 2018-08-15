% Test Data Annimation Plot
% Sangjun Lee
% 3/2/2018

% Main
figure;
plot(-5,0,'k',...
    [t nan t],[xEst_plot{k}(2,:)+3*xEst_plot{k}(3,:)...
    ,nan,xEst_plot{k}(2,:)-3*xEst_plot{k}(3,:)],'r--','LineWidth',lineW); 
axis([0,tEnd,-3,5]); grid on;
xlabel('time (s)')
ylabel(sprintf('S_{%d}(k)',k),'Fontsize',labelS);
title('Test Statistics')
%legend({'Residual','Threshold'},'Location','Southeast','FontSize',labelS)

ax(k) = gca; ax(k).FontSize = axisS;
h(k) = hgtransform('Parent',ax(k)); 
hold on
plot(t(1),xEst_plot{k}(1,1),'b','Parent',h(k),'LineWidth',lineW); 
hold off
txt(k) = text(t(1),xEst_plot{k}(1,1),num2str(xEst_plot{k}(1,1)),'Parent',h(k),...
    'VerticalAlignment','top','FontSize',axisS);
g(k) = animatedline('LineWidth',lineW,'Color','b');

% Draw
for i = 2:length(t)
    m{k} = makehgtform('translate',t(i)-t(1),...
        xEst_plot{k}(1,i)-xEst_plot{k}(1,1),0);
    h(k).Matrix = m{k};
    txt(k).String = num2str(xEst_plot{k}(1,i),'%.1f');
    addpoints(g(k),t(i),xEst_plot{k}(1,i));
    
    drawnow
end

