% Path following for the kinematic model of an Ackermann type car-like
% robot using transverse feedback linearization in continous time and
% under sampling. 

clc
clearvars

simTime = 5; 

T = 0.1;

l = 1; 
r = 2;

l1 = 1; 
l2 = 1; 
l3 = 1;

% a0 = l1*l2*l3; 
% a1 = (l1+l2)*l3+l1*l1; 
% a2 = l1*l2*l3;

a0 = 1; 
a1 = 4;
a2 = 3; 

k0 = 1; 

Change = 0;

a       = 2;
b       = 1.05*a;

pathType = 1;  % pathType 1 for circle, 2 for Casini oval;
refType = 1;   % specify type of velocity ref 1 for step changes, 2 for ramp

v0 = 1; 
v_ref = 2; 
ref0 = [0 0]';
m = 0.1;

A = [0 1 0;0 0 1;a0 a1 a2];
lambdatr = eig(A);
Ad  = [1 T 0;0 1 T;0 0 1]; % Approx. single rate normal form 
bd = [0;0; T];             % Approx. single rate normal form
lamdad1 = exp(lambdatr(1)*T); lamdad2 = exp(lambdatr(2)*T); lamdad3 = exp(lambdatr(3)*T);
pd = [lamdad1 lamdad2 lamdad3];
k1 = place(Ad,bd,pd);
Ac = [0 1 0;0 0 1;0 0 0]; bc = [0;0;1];
k2 = place(Ac,bc,lambdatr');


% start close to path in a "feasible pose"
if (pathType == 1)
    x0 = 3; 
    y0 = 2; 
    theta0 = pi;
    phi0 = 0;
else
    x0 = 1; 
    y0 = 1; 
    theta0 = pi/2;
    phi0 = 0;
end



%---------------------------------------------------------------------%
%--------------------------- SIMULATION ------------------------------%
%---------------------------------------------------------------------%

out = sim('AcermanDynExt.slx');


t = 0:10^-3:simTime;
   
figure 
% subplot(3,1,1);
subplot(4,1,1);


plot(out.x, out.y,'k', 'LineWidth', 2);
hold on; grid on;
plot(out.xe, out.ye,'LineWidth', 2);
plot(out.xsd, out.ysd,'LineWidth', 2);
plot(out.xmr, out.ymr,'b', 'LineWidth', 2);

l = legend('Continuous-time',  'ZOH emulation', 'Approx sampling', 'MR sampling' , 'Init conditions');
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = xlabel('$q_1(t)$'); 
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = ylabel('$q_2(t)$' ); 
set(l,'Interpreter','Latex'); l.FontSize = 20;


% plotting for 8 seconds





% subplot(3,1,2);
subplot(4,1,2);

plot(t(1:8001), out.ect(1:8001), 'k','LineWidth', 2);
hold on; grid on;
plot(t(1:8001), out.ee(1:8001),'LineWidth', 2);%
plot(t(1:8001), out.esd(1:8001), 'LineWidth', 2);
plot(t(1:8001) , out.emr(1:8001),'b', 'LineWidth', 2);


l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = ylabel('$\|p(t)\|_{\mathcal{C}}$' ); 
set(l,'Interpreter','Latex'); l.FontSize = 20;


subplot(4,1,3);
plot(t(1:8001), out.v(1:8001), 'k','LineWidth', 2);
hold on; grid on;
plot(t(1:8001), out.ve(1:8001),'LineWidth', 2);%
plot(t(1:8001), out.vsd(1:8001), 'LineWidth', 2);
plot(t(1:8001), out.vmr(1:8001),'b', 'LineWidth', 2);


l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = ylabel('$v(t)$' ); 
set(l,'Interpreter','Latex'); l.FontSize = 20;

% subplot(3,1,3);
subplot(4,1,4);

% plot(t(1:8001), out.uct(1:8001),'k', 'LineWidth', 2.5);
% hold on; grid on;
% plot(t(1:8001), out.ue(1:8001), 'LineWidth', 2.5);
% plot(t(1:8001), out.usd(1:8001), 'LineWidth', 2.5);
% plot(t(1:8001), out.umr(1:8001), 'b','LineWidth', 2.5);
% 
% l = xlabel('Time (s)'); 
% set(l,'Interpreter','Latex'); l.FontSize = 20;
% l = ylabel('$\sqrt{v^2 + \omega^2}$'); 
% set(l,'Interpreter','Latex'); l.FontSize = 20;
plot(t(1:8001), out.w(1:8001), 'k','LineWidth', 2);
hold on; grid on;
plot(t(1:8001), out.we(1:8001),'LineWidth', 2);%
plot(t(1:8001), out.wsd(1:8001), 'LineWidth', 2);
plot(t(1:8001), out.wmr(1:8001),'b', 'LineWidth', 2);


l = xlabel('Time (s)'); 
set(l,'Interpreter','Latex'); l.FontSize = 20;
l = ylabel('$\omega(t)$' ); 
set(l,'Interpreter','Latex'); l.FontSize = 20;




%---------------------------------------------------------------------%
%--------------------------- ANIMATIONS ------------------------------%
%---------------------------------------------------------------------%

% Draw_traj(out.x,out.y,out.theta, out.xe,out.ye,out.thetae,...
%     out.xmr, out.ymr,out.thetamr, out.xsd, out.ysd,out.thetasd,...
%     out.xmr_delta, out.ymr_delta,out.thetamr_delta, T);



function Draw_traj(x,y,theta,...
                   xe,ye, thetae, xmr, ymr,thetamr,...
                   xsd, ysd,thetasd, xmr_delta, ymr_delta, thetamr_delta, T)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
line_width = 1.5;
fontsize_labels = 14;

x_c_1 = [];  x_e_1 = [];  x_mr_1 = []; x_sd_1 = []; xmr_delta_1 = [];
y_c_1 = [];  y_e_1 = [];  y_mr_1 = []; y_sd_1 = []; ymr_delta_1 = [];

figure('Name','Path following trajectories')
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

unicycle_size=0.4;
vertices_unicycle_shape=unicycle_size*[[-0.25;-0.5;1/unicycle_size],...
    [0.7;0;1/unicycle_size],[-0.25;0.5;1/unicycle_size]];
faces_unicycle_shape=[1 2 3];


for k = 1:50:size(x,1)
    x1 = x(k); y1 = y(k); theta1 = theta(k); xe1 = xe(k);  ye1 = ye(k); thetae1 = thetae(k); 
    xmr1 = xmr(k);  ymr1 = ymr(k); thetamr1 = thetamr(k); xsd1 = xsd(k);  ysd1 = ysd(k);
    thetasd1 = thetasd(k); xmr_delta1 = xmr_delta(k);  ymr_delta1 = ymr_delta(k); thetamr_delta1 = thetamr_delta(k);
    x_c_1 = [x_c_1 x1]; x_e_1 = [x_e_1 xe1]; x_mr_1 = [x_mr_1 xmr1]; x_sd_1 = [x_sd_1 xsd1];
    y_c_1 = [y_c_1 y1]; y_e_1 = [y_e_1 ye1]; y_mr_1 = [y_mr_1 ymr1]; y_sd_1 = [y_sd_1 ysd1];
    xmr_delta_1 = [xmr_delta_1 xmr_delta1];  ymr_delta_1 = [ymr_delta_1 ymr_delta1];
    
    
    scatter(x(1),y(1),'k','diamond', 'LineWidth', 5);
    hold on;

    c = plot(x_c_1,y_c_1,'-k','linewidth',line_width);hold on % 
    M=[cos(theta1) -sin(theta1) x1; sin(theta1) cos(theta1)  y1;0 0 1]; 
    vertices_unicycle_shape_0=(M*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','k','EdgeColor','k','EraseMode','none');

    e = plot(x_e_1,y_e_1,'-r','linewidth',line_width);    
    Me=[cos(thetae1) -sin(thetae1) xe1; sin(thetae1) cos(thetae1)  ye1;0 0 1]; 
    vertices_unicycle_shape_0=(Me*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','r','EdgeColor','k','EraseMode','none');
    
    mr = plot(x_mr_1,y_mr_1,'-b','linewidth',line_width);    
    Mmr=[cos(thetamr1) -sin(thetamr1) xmr1; sin(thetamr1) cos(thetamr1)  ymr1;0 0 1]; 
    vertices_unicycle_shape_0=(Mmr*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','b','EdgeColor','k','EraseMode','none');
    
    mrd = plot(xmr_delta_1,ymr_delta_1,'-g','linewidth',line_width);    
    Mmrd=[cos(thetamr_delta1) -sin(thetamr_delta1) xmr_delta1; sin(thetamr_delta1) cos(thetamr_delta1)  ymr_delta1;0 0 1]; 
    vertices_unicycle_shape_0=(Mmrd*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','g','EdgeColor','k','EraseMode','none');
    
    sd = plot(x_sd_1,y_sd_1,'color','#D95319','linewidth',line_width);    
    Msd=[cos(thetasd1) -sin(thetasd1) xsd1; sin(thetasd1) cos(thetasd1)  ysd1;0 0 1]; 
    vertices_unicycle_shape_0=(Msd*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','#D95319','EdgeColor','k','EraseMode','none');
    
    hold off
    
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels);
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels);
    legend([c e mr mrd sd],'Continuous time','Emulation', 'MR Sampling', 'MR Sampling $y^\delta$','SR Sampling' ,'latex','FontSize',fontsize_labels);
    axis([-5 5 -5 5])
    pause(T/2)
    box on;
    grid on
    drawnow
end
end


