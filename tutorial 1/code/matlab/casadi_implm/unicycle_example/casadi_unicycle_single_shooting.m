% Unicycle with casadi single shooting

clc;
clearvars;
close all;

import casadi.*

T = 0.1; % delta, sampling period
Np = 3;  % prediction horizon, we assume Nc = Np

x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
v = SX.sym('v');
w = SX.sym('w');

states = [x;y;theta];
inputs = [v;w];
n      = length(states);
m      = length(inputs);

% optimization variables
u      = SX.sym('u',m,Np);
p      = SX.sym('p', n+n);
xpred  = SX.sym('xpred', n, Np + 1); % plus 1 for the init cond

% single shooting of the cost and constraints
xpred(:,1) = p(1:n); % initial state
for k = 1:Np
    kyn_next = [u(1,k)*cos(xpred(3,k));u(1,k)*sin(xpred(3,k));u(2,k)];
    xpred(:,k+1) = xpred(:,k) + T*kyn_next;
end

cost  = 0;
g     = [];
epsilon = 0.0;
Q     = diag([1 1 1]); R = epsilon*eye(m);
set_point   = p(n+1:end);
for k=1:Np
    cost = cost+(xpred(:,k)-set_point)'*Q*(xpred(:,k)-set_point) + u(:,k)'*R*u(:,k); % calculate obj
    g = [g ; xpred(1:2,k)];   
end
g     = [g ; xpred(1:2,Np+1)];  % g is of length Np+1


%% preparing the NLP
opt_var = reshape(u,m*Np,1);  % casadi expects a 1 dim vec for opti var
mp      = struct('f', cost, 'x', opt_var, 'g', g, 'p', p);  % mathematical program
opts    = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;
solver = casadi.nlpsol('solver', 'ipopt', mp, opts);

args = struct;
args.lbg = -inf;  
args.ubg = inf;
args.lbx(1:m:m*Np-1,1) = -inf; args.lbx(2:m:m*Np,1)   = -inf;
args.ubx(1:m:m*Np-1,1) = inf; args.ubx(2:m:m*Np,1)  = inf;

%% simulation

t0 = 0;
x0 = [0 ; 0 ; 0.0];    % initial condition.
xs = [2 ; 2  ; 0.0]; % Reference posture.
xx(:,1) = x0; % xx contains the history of states
t(1) = t0;
u0 = zeros(Np,m);
sim_tim = 10; 

mpciter = 0;
xx1 = [];
u_cl=[];
% value.
main_loop = tic;
while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    args.x0 = reshape(u0',m*Np,1); % initial value of the optimization variables
    %tic
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);    
    %toc
    u = reshape(full(sol.x)',m,Np)';
    pred = prediction(x0, u', Np, T); % compute optimal solution 
    xx1(:,1:3,mpciter+1)= full(pred)';
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    [t0, x0, u0] = plant(T, t0, x0, u); % get the initialization of the next optimization step
    xx(:,mpciter+2) = x0;  
    mpciter = mpciter + 1;
end
main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2);
average_mpc_time = main_loop_time/(mpciter+1);

x = xx(1,:)'; 
y = xx(2,:)';
theta  = xx(3,:)'; 


Draw_traj(x,y,theta, T);


%% animations
function Draw_traj(x,y,theta, T)
set(0,'DefaultAxesFontName', 'Times New Roman')
set(0,'DefaultAxesFontSize', 12)
line_width = 1.5;
fontsize_labels = 14;

x_c_1 = [];  y_c_1 = []; 

figure('Name','Path following trajectories')
set(gcf,'PaperPositionMode','auto')
set(gcf, 'Color', 'w');
set(gcf,'Units','normalized','OuterPosition',[0 0 0.55 1]);
vid = VideoWriter('unicycle.mp4');
open(vid);


unicycle_size=0.2;
vertices_unicycle_shape=unicycle_size*[[-0.25;-0.5;1/unicycle_size],...
    [0.7;0;1/unicycle_size],[-0.25;0.5;1/unicycle_size],[-0.25;-0.5;1/unicycle_size]];
faces_unicycle_shape=[1 2 3 4];


% p1 = 0:0.1:2*pi;
% p2 = 0:0.1:2*pi;


for k = 1:size(x)
    x1k = x(k); y1k = y(k); theta1k = theta(k); 
    x_c_1 = [x_c_1 x1k];  y_c_1 = [y_c_1 y1k]; 
    
%     plot(2*cos(p1),2*sin(p1), 'k--','linewidth',1.2);hold on % plot the reference trajectory
    hold on; grid on;
    scatter(2,2,'k','diamond', 'LineWidth', 5);

    c = plot(x_c_1,y_c_1,'-k','linewidth',line_width);hold on % 
    M=[cos(theta1k) -sin(theta1k) x1k; sin(theta1k) cos(theta1k)  y1k;0 0 1]; 
    vertices_unicycle_shape_0=(M*vertices_unicycle_shape)';
    vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
    patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','k','EdgeColor','k','EraseMode','none');
    hold off    

    ylabel('y-position (m)','interpreter','latex','FontSize',fontsize_labels);
    xlabel('x-position (m)','interpreter','latex','FontSize',fontsize_labels);
    legend([c],'Emulation','latex','FontSize',fontsize_labels);
    axis([-5 5 -5 5])
%     pause(T/2)
    box on;
    grid on  
    drawnow;
    frame = getframe(gcf);
    writeVideo(vid,frame);
end
end