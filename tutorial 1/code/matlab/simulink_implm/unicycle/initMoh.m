clc 
close all;
clearvars;

ref_type = 0;   % set to 0 for steering to (2, 2) on the x-y plane and set to 1 for tracking a straight-line with 0.5 forward velocity.

if ref_type == 0
    thetareference = 0;
    v0 = 0.0;
    ref0 = [2 2];
else
    thetareference = pi/4;
    v0 = 0.5;
    ref0 = [0 0];
end

scneario = 1;   % set to 0  to use preliminary transformation, and to 1 to avoid using it


% x01 = -1; x02 = 0; x03 = 0;
x01 = -1; x02 = 0; x03 = 0;
x0 = [x01;x02;x03];
simTime = 10;
delta = 1;
deltabar = delta/2;
t = 0:10^-3: simTime;
ts = 0:deltabar: simTime;
tref = 0:delta:simTime;

q = 1; % penalty on tracking error
r = 0.0; % penalty on controls

%% Nonlinear MPC controllers parameters
nx = 3;
ny = 3;
nu = 3;
nus = 2;
np = 3;
nc = np;
Ts = deltabar;

%% Controller object
Model.StateFcn = "ChainedForm_mr";
nlobj = nlmpc(nx, ny, nu);
nlobj.PredictionHorizon = np;
nlobj.ControlHorizon = nc;
nlobj.Model.StateFcn = "ChainedForm_mr";
nlobj.Model.IsContinuousTime = false;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
nlobj.Optimization.SolverOptions =  options;
%mpcobjSub.Optimization.UseSuboptimalSolution = true;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = @(x,u,Ts) [x(1);x(2); x(3)];
%nlobj.Jacobian.OutputFcn = @(x,u) [1 0 0;0 0 1];
nlobj.Jacobian.OutputFcn = @(x,u,Ts) eye(3);
nlobj.Weights.OutputVariables = q*[1 1 1];
nlobj.Weights.ManipulatedVariablesRate = r*[1 1 1]; %% try to play with weights 
u0 = [0;0;0];
validateFcns(nlobj,x0,u0,[],{Ts});
createParameterBus(nlobj,['unicycle_sd_mpc' '/Nonlinear MPC Controller'],'myBusObject',{Ts});


%% SR MPC controller object
Model.StateFcn = "ChainedForm_sr";
nlobjs = nlmpc(nx, ny, nus);
nlobjs.PredictionHorizon = np;
nlobjs.ControlHorizon = nc;
nlobjs.Model.StateFcn = "ChainedForm_sr";
nlobjs.Model.IsContinuousTime = false;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
nlobjs.Optimization.SolverOptions =  options;
%mpcobjSub.Optimization.UseSuboptimalSolution = true;
nlobjs.Model.NumberOfParameters = 0;
nlobjs.Model.OutputFcn = @(x,u) [x(1); x(2);x(3)];
nlobjs.Jacobian.OutputFcn = @(x,u) eye(3);
nlobjs.Weights.OutputVariables = q*[1 1 1];
nlobjs.Weights.ManipulatedVariablesRate = r*[1 1]; %% try to play with weights 
u0 = [0;0];
validateFcns(nlobjs,x0,u0,[],[]);


%% SR MPC and planned references controller object
Model.StateFcn = "ChainedForm_sr";
nlobjp = nlmpc(nx, ny, nus);
nlobjp.PredictionHorizon = np;
nlobjp.ControlHorizon = nc;
nlobjp.Model.StateFcn = "ChainedForm_sr";
nlobjp.Model.IsContinuousTime = false;
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
nlobjp.Optimization.SolverOptions =  options;
%mpcobjSub.Optimization.UseSuboptimalSolution = true;
nlobjp.Model.NumberOfParameters = 0;
nlobjp.Model.OutputFcn = @(x,u) [x(1); x(2);x(3)];
nlobjp.Jacobian.OutputFcn = @(x,u) eye(3);
nlobjp.Weights.OutputVariables = q*[1 1 1];
nlobjp.Weights.ManipulatedVariablesRate = r*[1 1]; %% try to play with weights 
u0 = [0;0];
validateFcns(nlobjp,x0,u0,[],[]);


%% Simulating and plotting



sim('unicycle_sd_mpc.slx');


x_mpc = xmpc(:,1); ympc = xmpc(:,2); thetampc = xmpc(:,3);
% xmr = xm(:,1); ymr = xm(:,2); thetamr = xm(:,3);
xsr = xs(:,1); ysr = xs(:,2); thetasr = xs(:,3);
x_p = xp(:,1); y_p = xp(:,2); theta_p = xp(:,3);



%---------------------------------------------------------------------%
%--------------------------- ANIMATIONS ------------------------------%
%---------------------------------------------------------------------%

Draw_traj(xsr,ysr,thetasr, xmpc(:,1),xmpc(:,2),xmpc(:,3), xp(:,1),xp(:,2),xp(:,3), ref_type, delta);

function Draw_traj(x,y,theta, x2,y2, theta2,x3, y3, theta3, ref_type, T)


set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
line_width = 1.5;
fontsize_labels = 14;

x_e_1 = []; x_p_1 = []; x_m_1 = [];
y_e_1 = []; y_p_1 = []; y_m_1 = [];
 


figure('Name','Path following trajectories')
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);

v = VideoWriter('unicycle_video.mp4');
% v.Quality   = 100;
% v.FrameRate = fR;
open(v);

if ref_type == 0
    unicycle_size=0.4;
else
    unicycle_size=0.15;
end

vertices_unicycle_shape   = unicycle_size*[[-0.25;-0.5;1/unicycle_size],...
                            [0.7;0;1/unicycle_size],[-0.25;0.5;1/unicycle_size],[-0.25;-0.5;1/unicycle_size]];
faces_unicycle_shape=[1 2 3 4];

% p1 = 0:0.1:2*pi;
% p2 = 0:0.1:2*pi;
p1   = 2*ones(1,size(x,1));
p2   = 2*ones(1, size(x,1));

for k = 1:50:size(x2)
    xe1 = x(k); ye1 = y(k); thetae1 = theta(k); xm1 = x2(k);  ym1 = y2(k); thetam1 = theta2(k); 
    xp1 = x3(k);  yp1 = y3(k); thetap1 = theta3(k); 

    x_e_1 = [x_e_1 xe1];   x_p_1 = [x_p_1 xp1]; x_m_1 = [x_m_1 xm1];
    y_e_1 = [y_e_1 ye1];   y_p_1 = [y_p_1 yp1]; y_m_1 = [y_m_1 ym1];

    if ref_type == 0
        scatter(p1(1),p2(1),'k','diamond', 'LineWidth', 5);        
    end

    hold on;
    scatter(x(1),y(1),'k','diamond', 'LineWidth', 5);

    e = plot(x_e_1,y_e_1,'k','linewidth',line_width);hold on % 
    M=[cos(thetae1) -sin(thetae1) xe1; sin(thetae1) cos(thetae1)  ye1;0 0 1]; 
    vertices_unicycle_shape_0=(M*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','k','EdgeColor','k','EraseMode','none');

    m = plot(x_m_1,y_m_1,'-r','linewidth',line_width);    
    Me=[cos(thetam1) -sin(thetam1) xm1; sin(thetam1) cos(thetam1)  ym1;0 0 1]; 
    vertices_unicycle_shape_0=(Me*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','r','EdgeColor','k','EraseMode','none');

    p = plot(x_p_1,y_p_1,'-b','linewidth',line_width);    
    Mp=[cos(thetap1) -sin(thetap1) xp1; sin(thetap1) cos(thetap1)  yp1;0 0 1]; 
    vertices_unicycle_shape_0=(Mp*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','b','EdgeColor','k','EraseMode','none');
    
    hold off
    
    ylabel('$y$-position (m)','interpreter','latex','FontSize',fontsize_labels);
    xlabel('$x$-position (m)','interpreter','latex','FontSize',fontsize_labels);

    legend([e m p],'Emulation MPC', 'MR Prediction MPC' , 'MR planner MPC', 'latex','FontSize',fontsize_labels);

    axis([-3 3 -3 3])
    pause(T/2)
    box on;
    grid on
    drawnow
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end
end




