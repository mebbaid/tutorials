%% Example CASADI MPC for unicycle 
% multiple shooting with  NLP NMPC

clc
clear all


import casadi.*

delta  = 0.1; % sampling time
Np = 3; % prediction horizon
T  = delta;

% OCP problem parameters
x1 = SX.sym('x'); x2 = SX.sym('y'); x3 = SX.sym('theta');
states = [x1; x2 ; x3];
n_states = length(states);
u1 = SX.sym('v'); u2 = SX.sym('omega'); 
controls = [u1; u2];
n_controls = length(controls);

% kinematics of diff drive for prediction model

f    = [0;0;0];
g1   = [cos(x3);sin(x3);0];
g2   = [0;0;1];
rhs  = f + g1*u1 + g2*u2;

fd   = Function('f',{states,controls},{rhs});  % nonlinear mapping function object

% setting up OCP

U   = SX.sym('U',n_controls,Np);  % decision variables along horizon
P   = SX.sym('P',n_states+n_states*Np); % parameters including init and ref values of nlp problem
X   = SX.sym('X',n_states,Np+1);     % states along the horizon

% computing obj and equality constraints
obj = 0; % obj function
g   = [];% constraints vector 
epsilon = 0.001;
Q   = eye(n_states); 
R   = epsilon*eye(n_controls); % weights on controls

st  = X(:,1);
g   = [g; st-P(1:n_states)]; % init condition constraints
for k = 1:Np
   st =  X(:,k); con = U(:,k);
   obj = obj+(st-P(n_states*k+1:(k+1)*n_states))'*Q*(st-P(n_states*k+1:(k+1)*n_states)) + con'*R*con;  % quadratic cost along horizon  
   st_next = X(:,k+1);
   f_value = fd(st,con);
   st_next_euler = st+T*f_value;
   g  = [g; st_next - st_next_euler]; % compute path constraints at every point along horizon
end

OPT_variables = [reshape(X,n_states*(Np+1),1) ;reshape(U,n_controls*Np,1)]; % states are also opt variables in multi shooting
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);  % nlp problem
opts = struct; % solver options
opts.ipopt.max_iter = 1000;
opts.ipopt.print_level = 1;
opts.ipopt.acceptable_tol = 1e-8;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct;  % bounds on the optimization variable
args.lbx(n_states*(Np+1)+1:n_controls:n_states*(Np+1)+n_controls*Np,1) = -inf; 
args.lbx(n_states*(Np+1)+2:n_controls:n_states*(Np+1)+n_controls*Np,1) = -inf; % bounds on control along horizon
args.ubx(n_states*(Np+1)+1:n_controls:n_states*(Np+1)+n_controls*Np,1) = inf;  
args.ubx(n_states*(Np+1)+2:n_controls:n_states*(Np+1)+n_controls*Np,1) = inf;

args.lbx(1:n_states:n_states*(Np+1),1) = -inf; % lower bound on x
args.ubx(1:n_states:n_states*(Np+1),1) = inf; % upper bound on x
args.lbx(2:n_states:n_states*(Np+1),1) = -inf; % lower bound on y
args.ubx(2:n_states:n_states*(Np+1),1) = inf; % upper bound on y
args.lbx(3:n_states:n_states*(Np+1),1) = -inf; % lower bound on theta
args.ubx(3:n_states:n_states*(Np+1),1) = inf; % upper bound on theta



args.lbg(1:n_states*(Np+1)) = 0;   % equality constraints
args.ubg(1:n_states*(Np+1)) = 0;


% Simulation loop
t0 = 0;
x0 = [0;0;0]; %  init posture
ref_const = [2;2;0];

xx(:,1) = x0;  % history of states
t(1) = t0; % time 
u0 = zeros(Np,n_controls); % two controls along horizon
X0 = repmat(x0,1,Np+1); % init of state decision variables


simTime = 15;
mpcitr = 0; % start mpc
xx1 = [];
u_cl = [];
n_ref = [];
dstr = [];

j = 0.1;

odeoptions = odeset('RelTol',10^-5, 'AbsTol',1e-7);

%% main simulation loop
while(norm((x0-ref_const),2) > 1e-2 && mpcitr < simTime/T)
   args.p(1:n_states) = x0; %values of parameters of opt problem at begining
   current_time = mpcitr*T;
   for k = 1:Np
      t_predict = current_time + (k-1)*T; % predicted time instant
       %---------------------------------------------------------------%
       %--------------- INSERT MULTIRATE REF PLANNER HERE -------------%
       %---------------------------------------------------------------%
      ref = ref_const;      
      xref = ref(1); yref = ref(2); thetaref = ref(3);
      args.p(n_states*k+1:n_states*(k+1)) = [xref, yref, thetaref];
   end   
   
   n_ref = [n_ref args.p(n_states*k+1:n_states*(k+1))'];
   args.x0 = [reshape(X0',n_states*(Np+1),1);reshape(u0',n_controls*Np,1)]; % init value of opt variable
   sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx, 'lbg', args.lbg, ...
             'ubg', args.ubg, 'p', args.p);
   u   = reshape(full(sol.x(n_states*(Np+1)+1:end))',n_controls,Np)'; % opt controls in 2*Np 
   xx1(:,1:n_states,mpcitr+1) = reshape(full(sol.x(1:n_states*(Np+1)))',n_states,Np+1)';
   u_cl = [u_cl;u(1,:)];
   t(mpcitr+1) = t0;
   [t0, x0, u0] = plant(T,t0, x0, u);  % apply control and update opt problem init values x0,u0,t0
   xx(:,mpcitr+2) = x0;
   X0 = reshape(full(sol.x(1:n_states*(Np+1)))',n_states,Np+1)';
   X0 = [X0(2:end,:);X0(end,:)];
   mpcitr = mpcitr + 1;  

end

x = xx(1,:)'; refx = n_ref(1,:)';
y = xx(2,:)'; refy = n_ref(2,:)';
theta  = xx(3,:)'; reftheta = n_ref(3,:)';


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
vid = VideoWriter('unicycle');
vid.Quality   = 100;
vid.FrameRate = 30;
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
    pause(T/2)
    box on;
    grid on  
    drawnow;
    frame = getframe(gcf);
    writeVideo(vid,frame);
end

close(vid);
end

