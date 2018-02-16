close all; clc;

LineW = 2.0;
FontS_axis = 15;
FontS_label = 18;

load data/t1_r1; load data/t1_r2; load data/t1_r3; 
load data/t2_r2; load data/t2_r3;

%xSigEst_r1 = t1_r1;
xSigEst_r1 = [xSigEst(att_state,:) ; fMean ; fSTD];
xSigEst_r2 = t2_r2;
xSigEst_r3 = t2_r3;



%% 1st Object
subplot(3,1,1)
plot(-5,0,'k',...
    [t nan t],[xSigEst_r1(2,:)+3*xSigEst_r1(3,:)...
    ,nan,xSigEst_r1(2,:)-3*xSigEst_r1(3,:)],'r--','LineWidth',LineW); 
axis([0,60,-1.1,1.1]); grid on;
ylabel('S_{R1}(k)','Fontsize',FontS_label);
title('Heading Angle')
legend({'Residual','Threshold'},'Location','Northeast','FontSize',FontS_label)

ax_r1 = gca; ax_r1.FontSize = FontS_axis;
h_r1 = hgtransform('Parent',ax_r1); 
hold on
plot(t(1),xSigEst_r1(1,1),'b','Parent',h_r1,'LineWidth',LineW); 
hold off
text_r1 = text(t(1),xSigEst_r1(1,1),num2str(xSigEst_r1(1,1)),'Parent',h_r1,...
    'VerticalAlignment','top','FontSize',14);
g_r1 = animatedline;


%% 2nd Object
subplot(3,1,2)
plot(-5,0,'k',...
    [t nan t],[xSigEst_r2(2,:)+3*xSigEst_r2(3,:)...
    ,nan,xSigEst_r2(2,:)-3*xSigEst_r2(3,:)],'r--','LineWidth',LineW); 
axis([0,60,-1.1,1.1]); grid on;
ylabel('S_{R2}(k)','Fontsize',FontS_label);

ax_r2 = gca; ax_r2.FontSize = FontS_axis;
h_r2 = hgtransform('Parent',ax_r2); 
hold on
plot(t(1),xSigEst_r2(1,1),'b','Parent',h_r2,'LineWidth',LineW); 
hold off
text_r2 = text(t(1),xSigEst_r2(1,1),num2str(xSigEst_r2(1,1)),'Parent',h_r2,...
    'VerticalAlignment','top','FontSize',14);
g_r2 = animatedline;


%% 3rd Object
subplot(3,1,3)
plot(-5,0,'k',...
    [t nan t],[xSigEst_r3(2,:)+3*xSigEst_r3(3,:)...
    ,nan,xSigEst_r3(2,:)-3*xSigEst_r3(3,:)],'r--','LineWidth',LineW);  
axis([0,60,-1.1,1.1]); grid on;
xlabel('time (s)','Fontsize',FontS_label); ylabel('S_{R3}(k)','Fontsize',FontS_label);

ax_r3 = gca; ax_r3.FontSize = FontS_axis;
h_r3 = hgtransform('Parent',ax_r3); 
hold on
plot(t(1),xSigEst_r3(1,1),'b','Parent',h_r3,'LineWidth',LineW); 
hold off
text_r3 = text(t(1),xSigEst_r3(1,1),num2str(xSigEst_r3(1,1)),'Parent',h_r3,...
    'VerticalAlignment','top','FontSize',14);
g_r3 = animatedline;


%% Main Loop
for k = 2:ct
    % 1st Object
    m_r1 = makehgtform('translate',t(k)-t(1),...
        xSigEst_r1(1,k)-xSigEst_r1(1,1),0);
    h_r1.Matrix = m_r1;
    text_r1.String = num2str(xSigEst_r1(1,k),'%.1f');
    addpoints(g_r1,t(k),xSigEst_r1(1,k));
    
    % 2nd Object
    m_r2 = makehgtform('translate',t(k)-t(1),...
        xSigEst_r2(1,k)-xSigEst_r2(1,1),0);
    h_r2.Matrix = m_r2;
    text_r2.String = num2str(xSigEst_r2(1,k),'%.1f');
    addpoints(g_r2,t(k),xSigEst_r2(1,k));
    
    % 3rd Object
    m_r3 = makehgtform('translate',t(k)-t(1),...
        xSigEst_r3(1,k)-xSigEst_r3(1,1),0);
    h_r3.Matrix = m_r3;
    text_r3.String = num2str(xSigEst_r3(1,k),'%.1f');
    addpoints(g_r3,t(k),xSigEst_r3(1,k));
    
    drawnow
end

