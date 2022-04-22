%% Car_Like plots

function CarPlots(x,y, theta)

x=x(:,1);
y=y(:,1);
theta=theta(:,1);

x_0 = x(1,1);
y_0 = y(1,1);
theta_0 = theta(1,1);

figure(1);
%subplot(2,2,1)
hold on;
axis equal;
set(gca,'fontname','Times','fontsize',16,'fontweight','normal');box on;

% setup unicycle shape

unicycle_size=0.5;
vertices_unicycle_shape=unicycle_size*[[-0.25;-0.5;1/unicycle_size],...
    [0.7;0;1/unicycle_size],[-0.25;0.5;1/unicycle_size]];
faces_unicycle_shape=[1 2 3];

% plot unicycle initial configuration

M=[cos(theta_0) -sin(theta_0) x_0; sin(theta_0) cos(theta_0)  y_0;0 0 1]; 
vertices_unicycle_shape_0=(M*vertices_unicycle_shape)';
vertices_unicycle_shape_0=vertices_unicycle_shape_0(:,1:2);
% patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
%     'FaceColor','none','EdgeColor','k','EraseMode','none');
patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_0,...
    'FaceColor','none','EdgeColor','k','EraseMode','none');

% plot unicycle final configuration

x=x(:,1);
y=y(:,1);
theta=theta;
x_f=x(length(x));
y_f=y(length(x));
theta_f=theta(length(x));

M=[cos(theta_f) -sin(theta_f) x_f; sin(theta_f) cos(theta_f)  y_f;0 0 1];
vertices_unicycle_shape_f=(M*vertices_unicycle_shape)';
vertices_unicycle_shape_f=vertices_unicycle_shape_f(:,1:2);
patch('Faces',faces_unicycle_shape,'Vertices',vertices_unicycle_shape_f,...
    'FaceColor','none','EdgeColor','k','EraseMode','none');

% plot trajectory

plot(x,y,'k')
xlabel('[m]');ylabel('[m]');

range=axis;
incr=0.05;
range(1)=range(1)-(range(2)-range(1))*incr;
range(2)=range(2)+(range(2)-range(1))*incr;
range(3)=range(3)-(range(4)-range(3))*incr;
range(4)=range(4)+(range(4)-range(3))*incr;

range_abs=abs(range);
max_range=max(range_abs);
range=[-max_range max_range -max_range max_range];
axis(range);

line([range(1) range(2)],[0 0],'Color','k','LineStyle','-','LineWidth',0.01)
line([0 0],[range(3) range(4)],'Color','k','LineStyle','-','LineWidth',0.01)

% figure(2);
% %subplot(4,2,1); hold on;
% subplot(2,1,1); hold on;
% plot(t,v,'k')
% set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
% ylabel('[m/s]');
% xlabel('[s]');
% title('driving velocity')
% box on;
% range=axis;
% incr=0.05;
% range(4)=range(4)+(range(4)-range(3))*incr;
% axis(range);
% 
% %subplot(4,2,3); hold on;
% subplot(2,1,2); hold on;
% plot(t,omega,'k')
% set(gca,'fontname','Times','fontsize',12,'fontweight','normal');
% ylabel('[rad/s]');
% xlabel('[s]');
% title('steering velocity')
% box on;
% range=axis;
% incr=0.05;
% range(4)=range(4)+(range(4)-range(3))*incr;
% axis(range);