% Test Data Annimation Plot
% Sangjun Lee
% 3/2/2018

% Main
figure;
title('Position Tracking')
grid on; axis([-1 1 -1 1]);
xlabel('x (m)','FontSize',labelS); ylabel('y (m)','FontSize',labelS)
color = {'r','g','b','y','m','g','k','m','k','r'};

hold on
for k = 1:N
    ax(k) = gca; ax(k).FontSize = axisS;
    h(k) = hgtransform('Parent',ax(k));
    plot(data.pose(1,k),data.pose(2,k),'Parent',h(k),'LineWidth',lineW);
    txt(k) = text(data.pose(1,k)+0.03,data.pose(2,k)+0.03,num2str(k),...
        'Parent',h(k),'VerticalAlignment','top','FontSize',axisS);
    g(k) = animatedline('LineWidth',lineW,'Color',color{k});
end
hold off; 

% Draw
for i = 2:length(t)
    for k = 1:N
    m{k} = makehgtform('translate',data.pose(i*3-2,k)-data.pose(1,k),...
        data.pose(i*3-1,k)-data.pose(2,k),0);
    h(k).Matrix = m{k};
    txt(k).String = num2str(k);
    addpoints(g(k),data.pose(i*3-2,k),data.pose(i*3-1,k));
    
    drawnow update
    end
end

