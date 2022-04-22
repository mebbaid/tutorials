%% main loop preparing ode and OCP.
clc; clearvars;

% parameters
t0       = 0;
delta    = 0.1; % sampling period
np       = 4;   % prediction horizon
simTime  = 10;
planttol = 10^-5;
predtol  = delta;
nx       = 3;
ny       = nx;
nu       = 2;
Q        = diag([1 1 1]);   
R        = 0.0*eye(nu);
x0      = [0;0;0];
u0      = zeros(nu,np);
cost    = 0;
xr      = zeros(nx,1);   % reference

options    = optimoptions('fmincon','Display','iter','Algorithm','sqp');
odeoptions = odeset('RelTol',10^-5, 'AbsTol',1e-7);
saturation  = 0;
sat_constraint = 0;
if saturation == 1
    satValue = 15; % adjust this value to saturate inputs
else
    satValue = inf;
end

% nonlinear constraints
c_ineq = [];
c_eq   = [];

% linear constraints
A = []; Aeq = [];
B = []; Beq = [];

% bounds
if sat_constraint == 1
    lb = -satValue*ones(nu,np);
    ub = satValue*ones(nu,np);
else
    lb = [];
    ub = [];
end

% ref preparation  (in case of time varying ref)
tracking = 0;   % 0 for posture regulation, 1 for a ramp 
ref_const = [2;2;0];

% plant preparation
u = sym('u',[nu np], 'real');
x = zeros(nx,1);

% save data for plotting later
xtraj = x0;
ztraj = [];
utraj = [];
erms  = [];
deltaVpoly = [];
ref   = [];
PolyNmpciter = [];

% main loop
for i=0:delta:simTime
   xpred =predictionModel(u,x0,np,delta);
   xr = ref_gen(tracking,ref_const, i);
   cost = costFunc(u,xpred,[xr xr xr xr],Q,R,np);
   fun = matlabFunction(cost,'Vars',{u});
   % prepare and solve OCP
   [ct,fval,exitflag,fminconoutput] = fmincon(fun,u0,A,B,Aeq,Beq,lb,ub,[],options);
   % apply control, update states
   if saturation ==1
        for j = 1:nu
            ct(j,1) = min(satValue, max(ct(j,1), -satValue));   % implements saturation with script
        end
   end
   [s,x] = ode45(@(t,x) plant(i,x,ct(:,1)),[i, i+delta],x0,odeoptions);
   % update and store data
   x = x(length(x),:)';
   er = x(1:3)-xr(1:3);
   xpred(:,1) = x;
   xtraj = [xtraj x]; 
   utraj = [utraj ct(:,1)];
   erms = [erms sqrt(er'*er)];
   ref   = [ref xr];
   xr    = zeros(nx,1);
   x0         = x;
   PolyNmpciter = [PolyNmpciter fminconoutput.iterations];
%    u0         = ct;  %not always wise to initialize fmincon with prev control, might get stuck on unfeasible solution
end

x = xtraj(1,:)';  y = xtraj(2,:)';  theta  = xtraj(3,:)';
refx = ref(1,:)'; refy = ref(2,:)'; reftheta = ref(3,:)';
Draw_traj(x,y,theta);

%% animations
function Draw_traj(x,y,theta)
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
    box on;
    grid on  
%     drawnow;
    frame = getframe(gcf);
    writeVideo(vid,frame);
end

close(vid);
end


