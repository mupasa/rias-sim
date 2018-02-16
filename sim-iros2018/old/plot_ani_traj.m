close all; clc;

LineW = 2.0;
FontS_axis = 15;
FontS_label = 18;

%% Initial Plot
plot(-20,20,'r>',-20,20,'g<',-20,20,'b<',...
    r3_controller.Waypoints(2,1),r3_controller.Waypoints(2,2),'k*',...
    r1_traj(:,1), r1_traj(:,2),'w',...
    r2_traj(:,1), r2_traj(:,2),'w',...
    r3_traj(:,1), r3_traj(:,2),'w',...
    'LineWidth',LineW)
axis([-10,70,-10,70]); grid on;
xlabel('East (m)','Fontsize',FontS_label); 
ylabel('North (m)','Fontsize',FontS_label);
legend({'R1 actual trajectory','R2 actual trajectory','R3 actual trajectory',...
    'Rendezvous point'},'Location','Northeast','FontSize',FontS_label)

%% 1st Object
ax_r1 = gca; ax_r1.FontSize = FontS_axis;
h_r1 = hgtransform('Parent',ax_r1); 
hold on
plot(r1_traj(1,1),r1_traj(1,2),'r>','Parent',h_r1,'LineWidth',LineW); 
hold off
text_r1 = text(r1_traj(1,1),r1_traj(1,2),num2str(r1_traj(1,2)),'Parent',h_r1,...
    'VerticalAlignment','top','FontSize',14);
g_r1 = animatedline;

%% 2nd Object
ax_r2 = gca;
h_r2 = hgtransform('Parent',ax_r2); 
hold on
plot(r2_traj(1,1),r2_traj(1,2),'g<','Parent',h_r2,'LineWidth',LineW); 
hold off
text_r2 = text(r2_traj(1,1),r2_traj(1,2),num2str(r2_traj(1,2)),'Parent',h_r2,...
    'VerticalAlignment','top','FontSize',14);
g_r2 = animatedline;

%% 3rd Object
ax_r3 = gca;
h_r3 = hgtransform('Parent',ax_r3); 
hold on
plot(r3_traj(1,1),r3_traj(1,2),'b<','Parent',h_r3,'LineWidth',LineW); 
hold off
text_r3 = text(r3_traj(1,1),r3_traj(1,2),num2str(r3_traj(1,2)),'Parent',h_r3,...
    'VerticalAlignment','top','FontSize',14);
g_r3 = animatedline;


%% Main Loop
for k = 2:ct
    % 1st Object
    m_r1 = makehgtform('translate',r1_traj(k,1)-r1_traj(1,1),...
        r1_traj(k,2)-r1_traj(1,2),0);
    h_r1.Matrix = m_r1;
    text_r1.String = num2str([r1_traj(k,1);r1_traj(k,2)],'%.1f');
    addpoints(g_r1,r1_traj(k,1),r1_traj(k,2));
    
    % 2nd Object
    m_r2 = makehgtform('translate',r2_traj(k,1)-r2_traj(1,1),...
        r2_traj(k,2)-r2_traj(1,2),0);
    h_r2.Matrix = m_r2;
    text_r2.String = num2str([r2_traj(k,1);r2_traj(k,2)],'%.1f');
    addpoints(g_r2,r2_traj(k,1),r2_traj(k,2));
    
    % 3rd Object
    m_r3 = makehgtform('translate',r3_traj(k,1)-r3_traj(1,1),...
        r3_traj(k,2)-r3_traj(1,2),0);
    h_r3.Matrix = m_r3;
    text_r3.String = num2str([r3_traj(k,1);r3_traj(k,2)],'%.1f');
    addpoints(g_r3,r3_traj(k,1),r3_traj(k,2));
    
    drawnow
end

